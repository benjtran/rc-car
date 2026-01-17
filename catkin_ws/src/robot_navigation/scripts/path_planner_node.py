#!/usr/bin/env python3
import rospy
import tf
import tf.transformations as tft
import numpy as np
from std_msgs.msg import Float32
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
        rospy.Timer(rospy.Duration(0.5), self.update)
    
    def map_callback(self, msg):
        self.map = msg
        self.map_received = True
    
    def update(self, event):
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
            return

        x, y = trans[0], trans[1]
        yaw = tft.euler_from_quaternion(rot)[2]
        self.pose = x, y, yaw
        mx, my = self.world_to_map(x, y)

        if mx is None:
            rospy.logwarn("Robot outside map bounds")
            return

        rospy.loginfo(
            "Robot pose: world (%.2f, %.2f) Rotation (%.2f)",
            x, y, yaw
        )

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
    
class CostMap:
    def __init__(self, reader, robot_radius=0.20): # Radius in meters
        self.reader = reader
        while not rospy.is_shutdown() and not self.reader.map_received:
            rospy.loginfo_throttle(5, "Waiting for map before creating costmap...")
            rospy.sleep(0.5)
        self.inflation_radius = int(robot_radius / self.reader.map.info.resolution)

        self.costmap = np.zeros(
            (self.reader.map.info.height, self.reader.map.info.width),
            dtype=np.uint8
        )

        self. costmap_pub = rospy.Publisher("/costmap", OccupancyGrid, queue_size=1, latch=True)
        rospy.Timer(rospy.Duration(0.5), self.generate_costmap)
    
    def mark_obstacles(self):
        w = self.reader.map.info.width
        h = self.reader.map.info.height

        for y in range(h):
            for x in range(w):
                val = self.reader.map.data[y * w + x]

                if val == 100:
                    self.costmap[y, x] = 254
                elif val == -1:
                    self.costmap[y, x] = 50
                else:
                    self.costmap[y, x] = 0

    def inflate_obstacles(self):
        h, w = self.costmap.shape

        lethal = np.argwhere(self.costmap == 254)

        for y, x in lethal:
            for dy in range(-self.inflation_radius, self.inflation_radius + 1):
                for dx in range(-self.inflation_radius, self.inflation_radius + 1):
                    ny = y + dy
                    nx = x + dx

                    if ny < 0 or nx < 0 or ny >= h or nx >= w:
                        continue

                    dist = np.hypot(dx, dy)
                    if dist > self.inflation_radius:
                        continue

                    if self.costmap[ny, nx] < 253:
                        self.costmap[ny, nx] = 253
    
    def publish_costmap(self):
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        grid_msg.info.resolution = self.reader.map.info.resolution
        grid_msg.info.width =  self.reader.map.info.width
        grid_msg.info.height =  self.reader.map.info.height
        grid_msg.info.origin =  self.reader.map.info.origin  # same origin as SLAM map

        # Flatten NumPy array and convert to int8
        grid_msg.data = self.costmap.flatten().astype(np.int8).tolist()

        self.costmap_pub.publish(grid_msg)

    
    def generate_costmap(self, event):
        if self.reader.map is None:
            rospy.logwarn_throttle(5, "No map yet, skipping costmap generation")
            return

        if not hasattr(self, 'inflation_radius'):
            robot_radius = 0.38
            self.inflation_radius = int(robot_radius / self.reader.map.info.resolution)
            self.costmap = np.zeros(
                (self.reader.map.info.height, self.reader.map.info.width),
                dtype=np.uint8
            )
            self.costmap_pub = rospy.Publisher("/costmap", OccupancyGrid, queue_size=1, latch=True)

        self.mark_obstacles()
        self.inflate_obstacles()
        self.publish_costmap()


class AStarPlanner:
    def __init__(self, costmap):
        self.costmap = costmap
        self.h, self.w = costmap.shape
        self.TRAVERSABLE_COST = 0  # Only white space

    def plan(self, start, goal):
        import heapq

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


class Navigator:
    def __init__(self, costmap):
        self.costmap = costmap
        self.pub = rospy.Publisher('/vel_cmd', Float32, queue_size=10)
        self.frontier_pub = rospy.Publisher("/frontiers", MarkerArray, queue_size=1)
        self.path_pub = rospy.Publisher("/planned_path", Marker, queue_size=1)  # For RViz
        self.vel = 0.0
        rospy.Timer(rospy.Duration(1), self.update)
    
    def update_frontiers(self):
        if not self.costmap.reader.map_received:
            rospy.loginfo_throttle(5, "Waiting for map...")
            return []

        grid, w, h = self.get_grid()
        frontiers = self.find_frontiers(grid, w, h)
        world_frontiers = [self.grid_to_world(mx, my) for mx, my in frontiers]

        # Publish for RViz visualization
        self.publish_frontiers(world_frontiers)
        return world_frontiers

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

    def is_frontier_cell(self, grid, x, y):
        if grid[y, x] != 0:
            return False
        for dx in [-1,0,1]:
            for dy in [-1,0,1]:
                nx, ny = x + dx, y + dy
                if nx < 0 or ny < 0 or nx >= grid.shape[1] or ny >= grid.shape[0]:
                    continue
                if grid[ny, nx] == -1:
                    return True
        return False

    def find_frontiers(self, grid, w, h):
        frontiers = []
        for y in range(h):
            for x in range(w):
                if self.is_frontier_cell(grid, x, y):
                    frontiers.append((x, y))
        return frontiers

    def select_frontier_with_path(self, robot_x, robot_y):
        """
        Select the closest frontier that has a valid path using A*.
        Returns world coordinates (x, y) and the path.
        """
        world_frontiers = self.update_frontiers()
        if not world_frontiers:
            return None, []

        # Convert robot pose to grid
        start_mx, start_my = self.world_to_grid(robot_x, robot_y)

        # Try each frontier, closest first
        frontiers_sorted = sorted(world_frontiers, key=lambda f: np.hypot(f[0]-robot_x, f[1]-robot_y))[:5]

        astar = AStarPlanner(self.costmap.costmap)
        for fx, fy in frontiers_sorted:
            goal_mx, goal_my = self.world_to_grid(fx, fy)
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
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        for x, y in path_world:
            from geometry_msgs.msg import Point
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)
        self.path_pub.publish(marker)

    def update(self, event):
        x, y, yaw = self.costmap.reader.pose
        target, path_world = self.select_frontier_with_path(x, y)
        if target:
            rospy.loginfo(f"Next reachable frontier: {target} with path length {len(path_world)}")
            # Publish path for RViz
            self.publish_path(path_world)

            # Here you can feed path_world to your Pure Pursuit controller
            # self.pure_pursuit.follow_path(path_world)

        # self.vel = 8 if self.vel == 0.0 else 0.0
        # msg = Float32(data=self.vel)

        # self.pub.publish(msg)
        # rospy.loginfo("Published velocity: %.2f", self.vel)

def main():
    rospy.init_node('path_planner', anonymous=True)
    rospy.loginfo("Path planner node started!")

    reader = MapPoseReader()
    costmap = CostMap(reader=reader)
    nav = Navigator(costmap=costmap)
    rospy.spin()
    # pub = rospy.Publisher('/vel_cmd', Float32, queue_size=10)
    # rate = rospy.Rate(4)

    # vel = 0.0

    # while not rospy.is_shutdown():
    #     vel = 8 if vel == 0.0 else 0.0
    #     msg = Float32(data=vel)

    #     pub.publish(msg)
    #     rospy.loginfo("Published velocity: %.2f", vel)

    #     rate.sleep()

if __name__ == "__main__":
    main()
