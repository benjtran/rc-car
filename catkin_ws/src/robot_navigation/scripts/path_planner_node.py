#!/usr/bin/env python3
import rospy
import tf
import tf.transformations as tft
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import heapq

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
    
    def world_to_map(self, x, y):
        if not self.map: return None, None
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
            return False

        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                "map", "base_link", rospy.Time(0)
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
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
        self.safety_costmap = None 
        self.costmap_pub = rospy.Publisher(
            "/costmap", OccupancyGrid, queue_size=1, latch=True
        )

    def update(self):
        map_msg = self.reader.map
        if map_msg is None:
            return False

        w = map_msg.info.width
        h = map_msg.info.height
        res = map_msg.info.resolution
        inflation_cells = int(self.robot_radius / res)

        grid = np.array(map_msg.data, dtype=np.int16).reshape((h, w))
        costmap = np.zeros((h, w), dtype=np.uint8)

        obstacle_mask = (grid == 100)
        unknown_mask = (grid == -1)

        costmap[obstacle_mask] = 254 
        costmap[unknown_mask] = 50   

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
        self.safety_costmap = self.create_safety_costmap(costmap, inflation_cells)
        self.publish()
        return True
    
    def create_safety_costmap(self, costmap, inflation_cells):
        h, w = costmap.shape
        safety_map = np.copy(costmap)
        safety_padding = max(5, inflation_cells)
        
        obstacle_cells = np.argwhere(costmap >= 253)
        
        for oy, ox in obstacle_cells:
            y0 = max(0, oy - safety_padding)
            y1 = min(h, oy + safety_padding + 1)
            x0 = max(0, ox - safety_padding)
            x1 = min(w, ox + safety_padding + 1)
            
            for y in range(y0, y1):
                for x in range(x0, x1):
                    if safety_map[y, x] >= 253: continue
                    dist = np.hypot(y - oy, x - ox)
                    if dist <= safety_padding:
                        gradient_cost = int(250 - (dist / safety_padding) * 240)
                        safety_map[y, x] = max(safety_map[y, x], gradient_cost)
        return safety_map

    def publish(self):
        if self.costmap is None: return
        msg = OccupancyGrid()
        msg.header = Header(stamp=rospy.Time.now(), frame_id="map")
        msg.info = self.reader.map.info
        msg.data = self.costmap.flatten().astype(np.int8).tolist()
        self.costmap_pub.publish(msg)
    
    def is_robot_in_danger(self, robot_x, robot_y):
        if self.costmap is None: return False
        map_msg = self.reader.map
        res = map_msg.info.resolution
        mx = int((robot_x - map_msg.info.origin.position.x) / res)
        my = int((robot_y - map_msg.info.origin.position.y) / res)
        h, w = self.costmap.shape
        if mx < 0 or my < 0 or mx >= w or my >= h: return False
        
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                cx, cy = mx + dx, my + dy
                if 0 <= cx < w and 0 <= cy < h:
                    if self.costmap[cy, cx] == 254: return True
        return False
    
    def is_path_safe(self, start_x, start_y, end_x, end_y):
        """Checks if a straight line between two points crosses obstacles"""
        if self.costmap is None: return False
        
        map_msg = self.reader.map
        res = map_msg.info.resolution
        x0 = int((start_x - map_msg.info.origin.position.x) / res)
        y0 = int((start_y - map_msg.info.origin.position.y) / res)
        x1 = int((end_x - map_msg.info.origin.position.x) / res)
        y1 = int((end_y - map_msg.info.origin.position.y) / res)
        
        h, w = self.costmap.shape
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                if 0 <= x < w and 0 <= y < h:
                    if self.costmap[y, x] >= 253: return False
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                if 0 <= x < w and 0 <= y < h:
                    if self.costmap[y, x] >= 253: return False
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        return True

    def get_escape_direction(self, robot_x, robot_y, robot_yaw):
        map_msg = self.reader.map
        res = map_msg.info.resolution
        mx = int((robot_x - map_msg.info.origin.position.x) / res)
        my = int((robot_y - map_msg.info.origin.position.y) / res)
        h, w = self.costmap.shape
        best_direction = None
        max_free_distance = 0
        robot_front_direction = robot_yaw + np.pi
        
        for angle in np.linspace(0, 2*np.pi, 16, endpoint=False):
            free_distance = 0
            for dist in range(1, 20):
                test_x = mx + int(dist * np.cos(angle))
                test_y = my + int(dist * np.sin(angle))
                if test_x < 0 or test_y < 0 or test_x >= w or test_y >= h: break
                if self.costmap[test_y, test_x] < 253: free_distance += 1
                else: break
            
            angle_diff = abs(self.normalize_angle(angle - robot_front_direction))
            if angle_diff > np.pi / 2: free_distance *= 1.5 
            
            if free_distance > max_free_distance:
                max_free_distance = free_distance
                best_direction = angle
        
        if best_direction is None: best_direction = robot_yaw
        return best_direction
    
    def normalize_angle(self, angle):
        while angle > np.pi: angle -= 2 * np.pi
        while angle < -np.pi: angle += 2 * np.pi
        return angle

class AStarPlanner:
    def __init__(self, costmap, safety_costmap=None):
        self.costmap = costmap
        self.safety_costmap = safety_costmap if safety_costmap is not None else costmap
        self.h, self.w = costmap.shape

    def plan(self, start, goal):
        if start == goal: return [start]

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        def heuristic(a, b): return np.hypot(a[0] - b[0], a[1] - b[1])
        
        def get_traversal_cost(pos):
            x, y = pos
            val = self.costmap[y, x]
            if val == 254: return float('inf')
            base_cost = 50.0 if val == 253 else 1.0
            safety_val = self.safety_costmap[y, x]
            return base_cost + (safety_val * 0.1)

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
                    if dx == 0 and dy == 0: continue
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self.w and 0 <= ny < self.h:
                        neighbor = (nx, ny)
                        traversal_cost = get_traversal_cost(neighbor)
                        if traversal_cost == float('inf'): continue
                        move_cost = np.hypot(dx, dy) * traversal_cost
                        tentative_g = g_score[current] + move_cost
                        if neighbor not in g_score or tentative_g < g_score[neighbor]:
                            g_score[neighbor] = tentative_g
                            f = tentative_g + heuristic(neighbor, goal)
                            heapq.heappush(open_set, (f, neighbor))
                            came_from[neighbor] = current
        return []

class IncrementalController:
    def __init__(self, costmap, reader):
        self.costmap = costmap
        self.reader = reader
        self.min_motor_vel = 0.1
        self.position_tolerance = 0.05
        self.angle_tolerance = 0.08
        self.state = "IDLE"
        self.target_position = None
        self.target_yaw = None
        self.move_start_time = None
        
    def normalize_angle(self, angle):
        while angle > np.pi: angle -= 2 * np.pi
        while angle < -np.pi: angle += 2 * np.pi
        return angle
    
    def set_linear_target(self, current_x, current_y, current_yaw, distance):
        forward_direction = current_yaw + np.pi
        target_x = current_x + distance * np.cos(forward_direction)
        target_y = current_y + distance * np.sin(forward_direction)
        self.target_position = (target_x, target_y)
        self.target_distance = distance
        self.state = "MOVING"
        self.move_start_time = rospy.Time.now()
    
    def set_angular_target(self, current_yaw, delta_yaw):
        self.target_yaw = self.normalize_angle(current_yaw + delta_yaw)
        self.state = "TURNING"
        self.move_start_time = rospy.Time.now()
    
    def compute_command(self, current_x, current_y, current_yaw):
        if self.state == "IDLE": return [0.0, 0.0, 0.0, 0.0]
        
        elif self.state == "MOVING":
            target_x, target_y = self.target_position
            dist = np.hypot(target_x - current_x, target_y - current_y)
            elapsed = (rospy.Time.now() - self.move_start_time).to_sec()
            
            if elapsed > 2.0 or dist < self.position_tolerance:
                self.state = "IDLE"
                return [0.0, 0.0, 0.0, 0.0]
            
            vel = self.min_motor_vel if self.target_distance > 0 else -self.min_motor_vel
            return [vel, vel, vel, vel]
        
        elif self.state == "TURNING":
            angle_error = self.normalize_angle(self.target_yaw - current_yaw)
            elapsed = (rospy.Time.now() - self.move_start_time).to_sec()
            
            if elapsed > 2.0 or abs(angle_error) < self.angle_tolerance:
                self.state = "IDLE"
                return [0.0, 0.0, 0.0, 0.0]
            
            vel = self.min_motor_vel
            if angle_error > 0: return [-vel, vel, -vel, vel]
            else: return [vel, -vel, vel, -vel]
        
        return [0.0, 0.0, 0.0, 0.0]

class Navigator:
    def __init__(self, costmap, reader):
        self.costmap = costmap
        self.reader = reader
        self.frontier_pub = rospy.Publisher("/frontiers", MarkerArray, queue_size=1)
        self.path_pub = rospy.Publisher("/planned_path", Marker, queue_size=1)
        self.vel_pub = rospy.Publisher('/vel_cmd', Float32MultiArray, queue_size=10)
        
        self.current_path = []
        self.current_goal = None
        self.current_waypoint_idx = 0
        self.FRONTIER_WINDOW = 80
        
        self.controller = IncrementalController(costmap, reader)
        self.behavior_state = "PLANNING"
        self.escape_count = 0
        self.last_replan_time = rospy.Time.now()
        
        # Spinning vars (for recovery only)
        self.total_spin = 0.0
        self.last_yaw = None
        
        rospy.Timer(rospy.Duration(0.1), self.control_loop)
        rospy.Timer(rospy.Duration(0.5), self.planning_loop)

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
        frontiers = []
        xmin = max(1, rx - self.FRONTIER_WINDOW)
        xmax = min(w - 1, rx + self.FRONTIER_WINDOW)
        ymin = max(1, ry - self.FRONTIER_WINDOW)
        ymax = min(h - 1, ry + self.FRONTIER_WINDOW)

        min_dist_cells = int(0.5 / self.costmap.reader.map.info.resolution)

        for y in range(ymin, ymax, 2):
            for x in range(xmin, xmax, 2):
                if grid[y, x] != 0: continue 

                dist_sq = (x - rx)**2 + (y - ry)**2
                if dist_sq < min_dist_cells**2:
                    continue
                
                has_unknown_neighbor = False
                for dy in range(-1, 2):
                    for dx in range(-1, 2):
                        if grid[y+dy, x+dx] == -1:
                            has_unknown_neighbor = True
                            break
                    if has_unknown_neighbor: break
                
                if not has_unknown_neighbor: continue
                if self.costmap.costmap[y, x] > 50: continue 
                
                frontiers.append((x, y))
        return frontiers

    def select_frontier_with_path(self, robot_x, robot_y):
        if not self.costmap.reader.map_received: return None, []
        grid, w, h = self.get_grid()
        rx, ry = self.world_to_grid(robot_x, robot_y)
        if rx < 0 or ry < 0 or rx >= w or ry >= h: return None, []

        frontiers = self.find_frontiers(grid, w, h, rx, ry)
        world_frontiers = [self.grid_to_world(mx, my) for mx, my in frontiers]
        self.publish_frontiers(world_frontiers)

        if not world_frontiers: return None, []

        start_mx, start_my = rx, ry
        frontiers_sorted = sorted(
            world_frontiers,
            key=lambda f: np.hypot(f[0] - robot_x, f[1] - robot_y)
        )[:3]

        astar = AStarPlanner(self.costmap.costmap, self.costmap.safety_costmap)

        for fx, fy in frontiers_sorted:
            goal_mx, goal_my = self.world_to_grid(fx, fy)
            if goal_mx < 0: continue

            path_cells = astar.plan((start_mx, start_my), (goal_mx, goal_my))
            if path_cells:
                path_world = [self.grid_to_world(mx, my) for mx, my in path_cells]
                return (fx, fy), path_world

        return None, []

    def publish_frontiers(self, world_frontiers):
        markers = MarkerArray()
        for i, (x, y) in enumerate(world_frontiers):
            if i > 50: break
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            markers.markers.append(marker)
        self.frontier_pub.publish(markers)

    def publish_path(self, path_world):
        if not path_world: return
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.b = 1.0
        marker.color.a = 1.0
        from geometry_msgs.msg import Point
        for x, y in path_world:
            p = Point(x=x, y=y, z=0.0)
            marker.points.append(p)
        self.path_pub.publish(marker)
        
    def clear_path(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path"
        marker.id = 0
        marker.action = Marker.DELETE 
        self.path_pub.publish(marker)

    def control_loop(self, event):
        # Make decisions based off controller state, with escape having the highest priority
        x, y, yaw = self.reader.pose
    
        if self.behavior_state == "COMPLETED":
            msg = Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0])
            self.vel_pub.publish(msg)
            return

        if self.costmap.is_robot_in_danger(x, y):
            if self.behavior_state != "ESCAPING":
                rospy.logwarn("DANGER! STARTING ESCAPE")
                self.behavior_state = "ESCAPING"
                self.escape_count = 0
                self.controller.state = "IDLE"
            
            if self.escape_count > 20: 
                self.behavior_state = "PLANNING"
                return

            if self.controller.state == "IDLE":
                escape_dir = self.costmap.get_escape_direction(x, y, yaw)
                robot_front = yaw + np.pi
                angle_diff = self.controller.normalize_angle(escape_dir - robot_front)
                
                if abs(angle_diff) > np.pi / 2:
                    self.controller.set_linear_target(x, y, yaw, -0.15)
                elif abs(angle_diff) > 0.3:
                    turn = np.clip(angle_diff, -0.3, 0.3)
                    self.controller.set_angular_target(yaw, turn)
                else:
                    self.controller.set_linear_target(x, y, yaw, 0.10)
                self.escape_count += 1
            
            cmd = self.controller.compute_command(x, y, yaw)
            msg = Float32MultiArray(data=cmd)
            self.vel_pub.publish(msg)
            return

        elif self.behavior_state == "ESCAPING":
            rospy.loginfo("ESCAPED. Recovering...")
            self.behavior_state = "RECOVERY"
            self.last_yaw = yaw 
            self.total_spin = 0
            self.controller.state = "IDLE"

        if self.behavior_state == "RECOVERY":
            delta = self.controller.normalize_angle(yaw - self.last_yaw)
            self.total_spin += abs(delta)
            self.last_yaw = yaw

            if self.total_spin > 1.0: 
                self.behavior_state = "PLANNING"
                self.total_spin = 0
                return

            if self.controller.state == "IDLE":
                 self.controller.set_angular_target(yaw, 0.5)
            
            cmd = self.controller.compute_command(x, y, yaw)
            msg = Float32MultiArray(data=cmd)
            self.vel_pub.publish(msg)
            return

        if self.behavior_state == "FOLLOWING_PATH":
            if not self.current_path or self.current_goal is None:
                self.behavior_state = "PLANNING"
                return

            goal_x, goal_y = self.current_goal
            dist_to_goal = np.hypot(goal_x - x, goal_y - y)

            if dist_to_goal < 0.20:
                rospy.loginfo("GOAL REACHED. Planning next...")
                self.behavior_state = "PLANNING"
                self.controller.state = "IDLE"
                self.current_path = []
                return

            if self.controller.state == "IDLE":
                if self.current_waypoint_idx >= len(self.current_path):
                    self.behavior_state = "PLANNING"
                    return
                
                target_x, target_y = self.current_path[self.current_waypoint_idx]
                dist_to_waypoint = np.hypot(target_x - x, target_y - y)
                
                if dist_to_waypoint < 0.10:
                    self.current_waypoint_idx += 1
                    return
                
                angle_to_target = np.arctan2(target_y - y, target_x - x)
                robot_front = yaw + np.pi
                angle_diff = self.controller.normalize_angle(angle_to_target - robot_front)
                
                if abs(angle_diff) > 0.3:
                    turn = np.clip(angle_diff, -0.3, 0.3)
                    self.controller.set_angular_target(yaw, turn)
                else:
                    move_dist = min(0.15, dist_to_waypoint)
                    
                    # continuously check paths
                    if not self.costmap.is_path_safe(x, y, target_x, target_y):
                        rospy.logwarn("Forward path blocked by obstacle/inflation! Re-planning.")
                        self.behavior_state = "PLANNING"
                        self.controller.state = "IDLE"
                        return

                    self.controller.set_linear_target(x, y, yaw, move_dist)
            
            cmd = self.controller.compute_command(x, y, yaw)
            msg = Float32MultiArray(data=cmd)
            self.vel_pub.publish(msg)
            return

        msg = Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0])
        self.vel_pub.publish(msg)

    def planning_loop(self, event):
        if not self.costmap.update(): return
        x, y, yaw = self.reader.pose

        # check if goal has been discovered
        if self.behavior_state == "FOLLOWING_PATH" and self.current_goal:
            gx, gy = self.world_to_grid(self.current_goal[0], self.current_goal[1])
            if gx and gy:
                if self.costmap.costmap[gy, gx] != 50:
                    rospy.loginfo("Frontier discovered dynamically (Not Unknown)! Re-planning.")
                    self.behavior_state = "PLANNING"
                    self.controller.state = "IDLE"
        
        if self.behavior_state == "PLANNING":
            grid, w, h = self.get_grid()
            rx, ry = self.world_to_grid(x, y)
            all_frontiers = self.find_frontiers(grid, w, h, rx, ry)
            
            if not all_frontiers:
                rospy.loginfo("NO FRONTIERS DETECTED. MISSION COMPLETE!")
                self.clear_path() 
                self.behavior_state = "COMPLETED"
                return

            target, path_world = self.select_frontier_with_path(x, y)
            
            if target and path_world:
                self.current_goal = target
                self.current_path = path_world
                self.current_waypoint_idx = 0
                self.behavior_state = "FOLLOWING_PATH"
                rospy.loginfo("New path found: %d waypoints", len(path_world))
                self.publish_path(path_world)
                self.last_replan_time = rospy.Time.now()
            else:
                elapsed_since_plan = (rospy.Time.now() - self.last_replan_time).to_sec()
                if elapsed_since_plan > 2.0:
                    rospy.logwarn("No path found. Entering RECOVERY.")
                    self.behavior_state = "RECOVERY"
                    self.last_yaw = yaw 
                    self.total_spin = 0
                    self.controller.state = "IDLE"

class PathPlanner:
    def __init__(self):
        self.reader = MapPoseReader()
        self.costmap = CostMap(self.reader)
        self.navigator = Navigator(self.costmap, self.reader)
        rospy.Timer(rospy.Duration(0.1), self.update)

    def update(self, event):
        if not self.reader.map_received: return
        self.reader.update_pose()

def main():
    rospy.init_node('path_planner', anonymous=True)
    rospy.loginfo("Dynamic path planner started!")
    path_planner = PathPlanner()
    rospy.spin()

if __name__ == "__main__":
    main()