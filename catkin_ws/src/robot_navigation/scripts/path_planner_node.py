#!/usr/bin/env python3
import rospy
import tf
import tf.transformations as tft
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

class MapPoseReader:
    def __init__(self):
        self.map = None
        self.map_received = False
        self.pose = np.zeros(3)
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
    
    def map_callback(self, msg):
        self.map = msg
        self.map_received = True
        self.map_updated = True
    
    def world_to_map(self, x, y):
        origin = self.map.info.origin.position
        resolution = self.map.info.resolution
        width = self.map.info.width
        height = self.map.info.height

        mx = int((x - origin.x) / resolution)
        my = int((y - origin.y) / resolution)

        if mx < 0 or my < 0 or mx >= width or my >= height:
            return None, None

        return mx, my
    
    def update_pose(self):
        if not self.map_received:
            rospy.loginfo_throttle(5, "Waiting for map...")
            return

        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                "map",
                "base_link",
                rospy.Time(0)
            )
        except tf.Exception:
            rospy.logwarn_throttle(2, "TF not available")
            return False

        x, y = trans[0], trans[1]
        yaw = tft.euler_from_quaternion(rot)[2]
        self.pose = x, y, yaw
        return True

class CostMap:
    def __init__(self, reader, robot_radius=0.20):
        self.reader = reader
        self.robot_radius = robot_radius
        self.costmap = None
        self.costmap_pub = rospy.Publisher(
            "/costmap", OccupancyGrid, queue_size=1, latch=True
        )
        self.map_stamp = None

    def update(self):
        map_msg = self.reader.map
        if map_msg is None:
            return False

        # Always recompute on every call for real-time updates
        self.map_stamp = map_msg.header.stamp

        w = map_msg.info.width
        h = map_msg.info.height
        res = map_msg.info.resolution
        inflation_cells = int(self.robot_radius / res)

        grid = np.array(map_msg.data, dtype=np.int16).reshape((h, w))
        costmap = np.zeros((h, w), dtype=np.uint8)

        # Mark obstacles and unknowns
        obstacle_mask = (grid == 100)
        unknown_mask = (grid == -1)

        costmap[obstacle_mask] = 254
        costmap[unknown_mask] = 50

        # Inflate obstacles (NumPy-only, local)
        obstacle_indices = np.argwhere(obstacle_mask)

        for oy, ox in obstacle_indices:
            y0 = max(0, oy - inflation_cells)
            y1 = min(h, oy + inflation_cells + 1)
            x0 = max(0, ox - inflation_cells)
            x1 = min(w, ox + inflation_cells + 1)

            for y in range(y0, y1):
                for x in range(x0, x1):
                    if costmap[y, x] >= 254:
                        continue
                    if np.hypot(y - oy, x - ox) <= inflation_cells:
                        costmap[y, x] = 253

        self.costmap = costmap
        self.publish()
        return True

    def publish(self):
        msg = OccupancyGrid()
        msg.header = Header(stamp=rospy.Time.now(), frame_id="map")
        msg.info = self.reader.map.info
        msg.data = self.costmap.flatten().astype(np.int8).tolist()
        self.costmap_pub.publish(msg)
    
    def is_robot_in_danger(self, robot_x, robot_y):
        """
        Check if robot is currently in an inflated obstacle cell or too close to one.
        Returns True if robot needs to escape.
        """
        if self.costmap is None:
            return False
        
        map_msg = self.reader.map
        res = map_msg.info.resolution
        mx = int((robot_x - map_msg.info.origin.position.x) / res)
        my = int((robot_y - map_msg.info.origin.position.y) / res)
        
        h, w = self.costmap.shape
        if mx < 0 or my < 0 or mx >= w or my >= h:
            return False
        
        # Check current cell and immediate neighbors (3x3 area)
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                check_x = mx + dx
                check_y = my + dy
                
                if 0 <= check_x < w and 0 <= check_y < h:
                    # Check if in inflated obstacle (253) or actual obstacle (254)
                    if self.costmap[check_y, check_x] >= 253:
                        return True
        
        return False
    
    def get_escape_direction(self, robot_x, robot_y, robot_yaw):
        """
        Find the best direction to escape from obstacle.
        Returns angle in radians in the map frame to move towards.
        Robot front faces -X, so we need to find direction with most free space.
        """
        map_msg = self.reader.map
        res = map_msg.info.resolution
        mx = int((robot_x - map_msg.info.origin.position.x) / res)
        my = int((robot_y - map_msg.info.origin.position.y) / res)
        
        h, w = self.costmap.shape
        
        # Sample directions around robot in map frame
        best_direction = None
        max_free_distance = 0
        
        # Test 16 directions in map frame
        for angle in np.linspace(0, 2*np.pi, 16, endpoint=False):
            free_distance = 0
            # Check along this direction for free space
            for dist in range(1, 30):  # Check up to 30 cells away
                test_x = mx + int(dist * np.cos(angle))
                test_y = my + int(dist * np.sin(angle))
                
                if test_x < 0 or test_y < 0 or test_x >= w or test_y >= h:
                    break
                
                if self.costmap[test_y, test_x] == 0:  # Free space
                    free_distance += 1
                else:
                    break
            
            if free_distance > max_free_distance:
                max_free_distance = free_distance
                best_direction = angle
        
        if best_direction is None:
            # Emergency fallback: move opposite to current heading
            # Robot faces -X, so reverse means go in +X direction in map frame
            # Current heading in map frame
            best_direction = robot_yaw + np.pi
        
        rospy.loginfo("Escape direction: %.2f rad, free distance: %d cells", 
                     best_direction, max_free_distance)
        
        return best_direction


class AStarPlanner:
    def __init__(self, costmap):
        self.costmap = costmap
        self.h, self.w = costmap.shape
        self.TRAVERSABLE_COST = 0  # Only white space

    def plan(self, start, goal):
        import heapq

        if start == goal:
            return [start]

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        def heuristic(a, b):
            return np.hypot(a[0] - b[0], a[1] - b[1])

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]

            cx, cy = current
            for dx in [-1,0,1]:
                for dy in [-1,0,1]:
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self.w and 0 <= ny < self.h:
                        if self.costmap[ny, nx] != self.TRAVERSABLE_COST:
                            continue  # STRICT: only white space
                        neighbor = (nx, ny)
                        tentative_g = g_score[current] + np.hypot(dx, dy)
                        if neighbor not in g_score or tentative_g < g_score[neighbor]:
                            g_score[neighbor] = tentative_g
                            f = tentative_g + heuristic(neighbor, goal)
                            heapq.heappush(open_set, (f, neighbor))
                            came_from[neighbor] = current
        return []  # No path found
    
    def is_reachable(self, start, goal, max_iterations=500):
        """
        Quick reachability check using BFS with limited iterations.
        Returns True if goal is reachable from start through free space.
        Reduced max_iterations for faster real-time performance.
        """
        from collections import deque
        
        if start == goal:
            return True
        
        visited = set()
        queue = deque([start])
        visited.add(start)
        iterations = 0
        
        while queue and iterations < max_iterations:
            iterations += 1
            cx, cy = queue.popleft()
            
            # Check 8-connected neighbors
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    
                    nx, ny = cx + dx, cy + dy
                    
                    # Check bounds
                    if not (0 <= nx < self.w and 0 <= ny < self.h):
                        continue
                    
                    # Check if traversable (only free space)
                    if self.costmap[ny, nx] != self.TRAVERSABLE_COST:
                        continue
                    
                    neighbor = (nx, ny)
                    
                    # Found the goal
                    if neighbor == goal:
                        return True
                    
                    # Add to queue if not visited
                    if neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)
        
        return False


class PurePursuitController:
    """
    Pure Pursuit controller for 4-wheel skid-steer robot.
    Converts path following into differential drive commands, then into wheel velocities.
    Robot orientation: Front faces -X, drives forward in -X direction.
    Motor limits: Min 0.1 m/s, Max 0.3 m/s
    """
    def __init__(self, lookahead_distance=0.2, wheel_base=0.3, max_linear_vel=0.2, max_angular_vel=0.5):
        self.lookahead_distance = lookahead_distance
        self.wheel_base = wheel_base
        self.max_linear_vel = max_linear_vel  # Keep under 0.3 motor limit
        self.max_angular_vel = max_angular_vel  # Adjusted for 0.3 motor limit
        self.max_motor_vel = 0.3  # Hardware upper limit
        self.min_motor_vel = 0.1  # Hardware lower limit
        self.goal_threshold = 0.15  # Distance to consider goal reached
        
        self.lookahead_pub = rospy.Publisher("/lookahead_point", Marker, queue_size=1)

    def find_lookahead_point(self, path, robot_x, robot_y):
        """Find the point on the path at lookahead distance."""
        if not path or len(path) < 2:
            return None
        
        # Find the closest point on path first
        min_dist = float('inf')
        closest_idx = 0
        for i, (px, py) in enumerate(path):
            dist = np.hypot(px - robot_x, py - robot_y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Search forward from closest point for lookahead point
        for i in range(closest_idx, len(path)):
            px, py = path[i]
            dist = np.hypot(px - robot_x, py - robot_y)
            if dist >= self.lookahead_distance:
                self.publish_lookahead(px, py)
                return (px, py)
        
        # If no point at lookahead distance, return last point
        self.publish_lookahead(path[-1][0], path[-1][1])
        return path[-1]

    def publish_lookahead(self, x, y):
        """Visualize lookahead point in RViz."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.lookahead_pub.publish(marker)

    def compute_velocity_command(self, path, robot_x, robot_y, robot_yaw):
        """
        Compute wheel velocities to follow the path.
        Robot orientation: Front faces -X, Right side faces +Y
        Robot drives FORWARD (in -X direction) to follow path.
        Motor limits: Min 0.1 m/s, Max 0.3 m/s
        Returns: [FL, FR, RL, RR] wheel velocities in m/s, or None if goal reached.
        """
        if not path:
            return None
        
        # Check if we've reached the goal
        goal_x, goal_y = path[-1]
        dist_to_goal = np.hypot(goal_x - robot_x, goal_y - robot_y)
        if dist_to_goal < self.goal_threshold:
            rospy.loginfo("Goal reached!")
            return [0.0, 0.0, 0.0, 0.0]
        
        # Find lookahead point
        lookahead = self.find_lookahead_point(path, robot_x, robot_y)
        if lookahead is None:
            return None
        
        target_x, target_y = lookahead
        
        # Vector from robot to target in map frame
        dx = target_x - robot_x
        dy = target_y - robot_y
        
        # Angle to target in map frame
        angle_to_target = np.arctan2(dy, dx)
        
        # Robot's front direction in map frame
        # Since front faces -X, the front direction is yaw + pi
        robot_front_direction = robot_yaw + np.pi
        
        # Angle difference between where we want to go and where front is pointing
        angle_diff = angle_to_target - robot_front_direction
        
        # Normalize to [-pi, pi]
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi
        
        # Distance to target
        distance_to_target = np.hypot(dx, dy)
        
        # Pure pursuit: angular velocity proportional to angle error
        # Positive angle_diff means target is to the left, need to turn left (CCW)
        angular_vel = 2.0 * angle_diff  # Proportional control
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        # Linear velocity - move forward
        linear_vel = self.max_linear_vel
        
        # Slow down when turning sharply
        if abs(angle_diff) > 0.5:  # ~30 degrees
            linear_vel *= 0.6
        
        # Convert to wheel velocities for skid-steer
        # For skid-steer: v_left = v - ω*w/2, v_right = v + ω*w/2
        # Positive linear_vel = forward in -X direction
        # Positive angular_vel = turn left (CCW)
        v_left = linear_vel - angular_vel * self.wheel_base / 2.0
        v_right = linear_vel + angular_vel * self.wheel_base / 2.0
        
        # Apply motor limits with dead zone handling
        def apply_motor_limits(vel):
            if abs(vel) < 0.01:  # Essentially zero
                return 0.0
            elif abs(vel) < self.min_motor_vel:  # Below minimum but not zero
                return self.min_motor_vel * np.sign(vel)
            else:  # Normal operation
                return np.clip(vel, -self.max_motor_vel, self.max_motor_vel)
        
        v_left = apply_motor_limits(v_left)
        v_right = apply_motor_limits(v_right)
        
        rospy.loginfo_throttle(
            2,
            "Target angle: %.2f, Robot front: %.2f, Diff: %.2f, Linear: %.2f, Angular: %.2f",
            angle_to_target, robot_front_direction, angle_diff, linear_vel, angular_vel
        )
        
        # Return [FL, FR, RL, RR]
        return [v_left, v_right, v_left, v_right]


class Navigator:
    def __init__(self, costmap):
        self.costmap = costmap
        self.frontier_pub = rospy.Publisher("/frontiers", MarkerArray, queue_size=1)
        self.path_pub = rospy.Publisher("/planned_path", Marker, queue_size=1)
        self.vel_pub = rospy.Publisher('/vel_cmd', Float32MultiArray, queue_size=10)
        
        self.current_path = []
        self.current_goal = None
        self.FRONTIER_WINDOW = 80
        
        # Spinning behavior
        self.is_spinning = False
        self.spin_start_time = None
        self.spin_duration = 12.56  # 2*pi / 0.5 rad/s = ~12.56 seconds for 360 degrees
        self.spin_angular_velocity = 0.5  # rad/s - achievable with 0.15 m/s wheel speed
        self.spin_threshold = 0.3  # Start spinning when within 0.3m of goal
        
        # Escape behavior
        self.is_escaping = False
        self.escape_direction = None
        
        # Exploration state
        self.exploration_complete = False
        
        # Pure Pursuit controller with slower speeds
        self.controller = PurePursuitController(
            lookahead_distance=0.2,
            wheel_base=0.3,
            max_linear_vel=0.2,  # Within 0.3 motor limit
            max_angular_vel=0.5  # Adjusted for 0.3 motor limit
        )
        
        rospy.Timer(rospy.Duration(0.1), self.control_loop)  # 10 Hz control
        rospy.Timer(rospy.Duration(0.5), self.planning_loop)  # 2 Hz planning (faster updates)

    def get_grid(self):
        map_msg = self.costmap.reader.map
        w = map_msg.info.width
        h = map_msg.info.height
        grid = np.array(map_msg.data, dtype=np.int8).reshape((h, w))
        return grid, w, h

    def grid_to_world(self, mx, my):
        map_msg = self.costmap.reader.map
        x = map_msg.info.origin.position.x + mx * map_msg.info.resolution
        y = map_msg.info.origin.position.y + my * map_msg.info.resolution
        return (x, y)

    def world_to_grid(self, x, y):
        map_msg = self.costmap.reader.map
        mx = int((x - map_msg.info.origin.position.x) / map_msg.info.resolution)
        my = int((y - map_msg.info.origin.position.y) / map_msg.info.resolution)
        return mx, my

    def find_frontiers(self, grid, w, h, rx, ry):
        """
        Find frontier cells that are:
        1. Free space (grid value 0)
        2. Adjacent to unknown space (grid value -1)
        3. NOT in inflated obstacle areas (costmap value 0)
        4. Reachable from robot position through free space
        """
        frontiers = []
        xmin = max(1, rx - self.FRONTIER_WINDOW)
        xmax = min(w - 1, rx + self.FRONTIER_WINDOW)
        ymin = max(1, ry - self.FRONTIER_WINDOW)
        ymax = min(h - 1, ry + self.FRONTIER_WINDOW)

        # Create A* planner for reachability checks
        astar = AStarPlanner(self.costmap.costmap)
        robot_pos = (rx, ry)

        for y in range(ymin, ymax):
            for x in range(xmin, xmax):
                # Must be free space in the original map
                if grid[y, x] != 0:
                    continue
                
                # Must be adjacent to unknown space
                if not np.any(grid[y-1:y+2, x-1:x+2] == -1):
                    continue
                
                # Must be traversable in the costmap (not in inflated obstacle area)
                if self.costmap.costmap[y, x] != 0:
                    continue
                
                # Must be reachable from robot position through free space
                if not astar.is_reachable(robot_pos, (x, y)):
                    continue
                
                frontiers.append((x, y))
        
        return frontiers

    def select_frontier_with_path(self, robot_x, robot_y):
        if not self.costmap.reader.map_received:
            return None, []

        grid, w, h = self.get_grid()
        rx, ry = self.world_to_grid(robot_x, robot_y)

        if rx < 0 or ry < 0 or rx >= w or ry >= h:
            return None, []

        # Check if robot is in safe location
        if self.costmap.costmap[ry, rx] != 0:
            rospy.logwarn("Robot at (%d, %d) is not in free space (cost: %d)! Skipping frontier search.",
                         rx, ry, self.costmap.costmap[ry, rx])
            return None, []

        frontiers = self.find_frontiers(grid, w, h, rx, ry)
        world_frontiers = [self.grid_to_world(mx, my) for mx, my in frontiers]
        self.publish_frontiers(world_frontiers)

        if not world_frontiers:
            return None, []

        start_mx, start_my = self.world_to_grid(robot_x, robot_y)
        frontiers_sorted = sorted(
            world_frontiers,
            key=lambda f: np.hypot(f[0] - robot_x, f[1] - robot_y)
        )[:5]

        astar = AStarPlanner(self.costmap.costmap)

        for fx, fy in frontiers_sorted:
            goal_mx, goal_my = self.world_to_grid(fx, fy)
            if goal_mx < 0 or goal_my < 0:
                continue

            path_cells = astar.plan((start_mx, start_my), (goal_mx, goal_my))
            if path_cells:
                path_world = [self.grid_to_world(mx, my) for mx, my in path_cells]
                return (fx, fy), path_world

        return None, []

    def publish_frontiers(self, world_frontiers):
        markers = MarkerArray()
        for i, (x, y) in enumerate(world_frontiers):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.scale.x = 0.08
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            markers.markers.append(marker)
        self.frontier_pub.publish(markers)

    def publish_path(self, path_world):
        if not path_world:
            return
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        from geometry_msgs.msg import Point
        for x, y in path_world:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)
        self.path_pub.publish(marker)

    def control_loop(self, event):
        """High-frequency loop for path following control."""
        x, y, yaw = self.costmap.reader.pose
        
        # PRIORITY 1: Check if robot is in danger (in inflated obstacle)
        if self.costmap.is_robot_in_danger(x, y):
            if not self.is_escaping:
                rospy.logwarn("DANGER! Robot in inflated obstacle area - initiating escape!")
                self.is_escaping = True
                self.escape_direction = self.costmap.get_escape_direction(x, y, yaw)
            
            # Move in escape direction (forward in -X direction)
            # Robot: Front faces -X in map frame, Right faces +Y
            # Motor limits: Min 0.1 m/s, Max 0.3 m/s
            
            # The escape_direction is in map frame, robot heading is also in map frame
            # Robot's front is at yaw + pi (since front faces -X)
            robot_front_direction = yaw + np.pi
            
            # Calculate angle difference between robot's front and escape direction
            angle_diff = self.escape_direction - robot_front_direction
            # Normalize to [-pi, pi]
            while angle_diff > np.pi:
                angle_diff -= 2 * np.pi
            while angle_diff < -np.pi:
                angle_diff += 2 * np.pi
            
            rospy.logwarn("Robot yaw: %.2f, Front dir: %.2f, Escape dir: %.2f, Angle diff: %.2f",
                         yaw, robot_front_direction, self.escape_direction, angle_diff)
            
            # If we need to turn more than 90 degrees, it's faster to reverse
            if abs(angle_diff) > np.pi / 2:
                # Back up and turn
                escape_linear_vel = -0.15  # Reverse
                escape_angular_vel = 0.15 * np.sign(angle_diff)
                rospy.logwarn("Reversing to escape!")
            else:
                # Turn towards escape direction and move forward
                escape_linear_vel = 0.15  # Forward
                escape_angular_vel = 0.0
                
                # Turn towards escape direction if needed
                if abs(angle_diff) > 0.15:  # More than ~8 degrees off
                    escape_angular_vel = 0.4 * np.sign(angle_diff)
                    escape_linear_vel = 0.1  # Minimum speed while turning
            
            v_left = escape_linear_vel - escape_angular_vel * self.controller.wheel_base / 2.0
            v_right = escape_linear_vel + escape_angular_vel * self.controller.wheel_base / 2.0
            
            # Apply motor limits (min 0.1, max 0.3)
            def apply_motor_limits(vel):
                if abs(vel) < 0.01:  # Essentially zero
                    return 0.0
                elif abs(vel) < 0.1:  # Below minimum but not zero
                    return 0.1 * np.sign(vel)
                else:  # Normal operation
                    return np.clip(vel, -0.3, 0.3)
            
            v_left = apply_motor_limits(v_left)
            v_right = apply_motor_limits(v_right)
            
            msg = Float32MultiArray()
            msg.data = [v_left, v_right, v_left, v_right]
            self.vel_pub.publish(msg)
            rospy.logwarn("ESCAPING - Angle diff: %.2f rad, Velocities: [%.2f, %.2f, %.2f, %.2f]", 
                         angle_diff, *msg.data)
            return
        else:
            # Robot is safe - clear escape flag
            if self.is_escaping:
                rospy.loginfo("Escaped from danger zone - resuming normal operation")
                self.is_escaping = False
                self.escape_direction = None
        
        # PRIORITY 2: Handle spinning behavior
        if self.is_spinning:
            elapsed = (rospy.Time.now() - self.spin_start_time).to_sec()
            if elapsed < self.spin_duration:
                # Spin in place - calculate required wheel speeds for desired angular velocity
                # ω = (v_right - v_left) / wheel_base
                # For spin in place: v_right = -v_left
                # So: ω = 2*v / wheel_base, therefore v = ω * wheel_base / 2
                spin_wheel_speed = self.spin_angular_velocity * self.controller.wheel_base / 2.0
                
                # Ensure wheel speed is within motor limits [0.1, 0.3]
                spin_wheel_speed = max(0.1, min(0.3, spin_wheel_speed))
                
                msg = Float32MultiArray()
                # Left wheels backward, right wheels forward for CCW rotation
                msg.data = [-spin_wheel_speed, spin_wheel_speed, -spin_wheel_speed, spin_wheel_speed]
                self.vel_pub.publish(msg)
                
                # Calculate actual angular velocity achieved
                actual_omega = 2.0 * spin_wheel_speed / self.controller.wheel_base
                
                rospy.loginfo("Spinning: %.1f%% complete (%.1f/%.1f sec) - Wheel speed: %.2f m/s, ω: %.2f rad/s", 
                             (elapsed / self.spin_duration) * 100, elapsed, self.spin_duration,
                             spin_wheel_speed, actual_omega)
                return
            else:
                # Finished spinning
                self.is_spinning = False
                self.spin_start_time = None
                rospy.loginfo("Finished 360 spin!")
                # Clear path so we can find next frontier
                self.current_path = []
                self.current_goal = None
                return
        
        # PRIORITY 3: Check if we're close to goal and should start spinning
        if self.current_goal is not None:
            goal_x, goal_y = self.current_goal
            dist_to_goal = np.hypot(goal_x - x, goal_y - y)
            
            rospy.loginfo_throttle(2, "Distance to goal: %.2f m (threshold: %.2f m)", 
                                  dist_to_goal, self.spin_threshold)
            
            if dist_to_goal < self.spin_threshold and not self.is_spinning:
                rospy.loginfo("Close to frontier! Starting 360 degree scan...")
                self.is_spinning = True
                self.spin_start_time = rospy.Time.now()
                return
        
        # PRIORITY 4: Normal path following
        if not self.current_path:
            # No path, stop
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0, 0.0, 0.0]
            self.vel_pub.publish(msg)
            return
        
        # Compute wheel velocities
        wheel_vels = self.controller.compute_velocity_command(
            self.current_path, x, y, yaw
        )
        
        if wheel_vels is None:
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0, 0.0, 0.0]
            self.vel_pub.publish(msg)
            return
        
        # Publish velocities
        msg = Float32MultiArray()
        msg.data = wheel_vels
        self.vel_pub.publish(msg)
        
        rospy.loginfo_throttle(
            1,
            "Following path - Velocities [FL FR RL RR]: [%.2f, %.2f, %.2f, %.2f]",
            *wheel_vels
        )

    def planning_loop(self, event):
        """Planning loop - now runs at 2 Hz for faster updates."""
        if not self.costmap.update():
            return

        x, y, yaw = self.costmap.reader.pose
        
        # Don't plan while spinning or escaping
        if self.is_spinning or self.is_escaping:
            return
        
        # Always try to update frontiers and path for real-time visualization
        target, path_world = self.select_frontier_with_path(x, y)
        
        if target and path_world:
            # Update path if we found a new/better one
            self.current_goal = target
            self.current_path = path_world
            self.exploration_complete = False
            rospy.loginfo_throttle(
                2,
                "Path updated to frontier: (%.2f, %.2f) with %d waypoints",
                target[0], target[1], len(path_world)
            )
            self.publish_path(path_world)
        else:
            # No more frontiers found
            if not self.exploration_complete:
                rospy.loginfo("=" * 60)
                rospy.loginfo("EXPLORATION COMPLETE - No more reachable frontiers!")
                rospy.loginfo("=" * 60)
                self.exploration_complete = True
                # Stop the robot
                msg = Float32MultiArray()
                msg.data = [0.0, 0.0, 0.0, 0.0]
                self.vel_pub.publish(msg)
                # Clear current path
                self.current_path = []
                self.current_goal = None


class PathPlanner:
    def __init__(self):
        self.reader = MapPoseReader()
        self.costmap = CostMap(self.reader)
        self.navigator = Navigator(self.costmap)
        rospy.Timer(rospy.Duration(0.1), self.update)  # 10 Hz for real-time updates

    def update(self, event):
        if not self.reader.map_received:
            return
        if not self.reader.update_pose():
            return
        # Costmap updates in planning loop now


def main():
    rospy.init_node('path_planner', anonymous=True)
    rospy.loginfo("Path planner node started!")
    
    path_planner = PathPlanner()
    
    rospy.spin()

if __name__ == "__main__":
    main()