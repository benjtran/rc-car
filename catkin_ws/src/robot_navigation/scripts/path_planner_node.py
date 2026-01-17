#!/usr/bin/env python3
import rospy
import tf
import tf.transformations as tft
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid

class MapPoseReader:
    def __init__(self):
        self.map = None
        self.map_received = False
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
    def __init__(self, reader, robot_radius=0.20): # Robot radius in meters
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


class Navigator:
    def __init__(self):
        self.pub = rospy.Publisher('/vel_cmd', Float32, queue_size=10)
        self.vel = 0.0
        rospy.Timer(rospy.Duration(0.5), self.update)
    
    def update(self, msg):
        self.vel = 8 if self.vel == 0.0 else 0.0
        msg = Float32(data=self.vel)

        self.pub.publish(msg)
        rospy.loginfo("Published velocity: %.2f", self.vel)

def main():
    rospy.init_node('path_planner', anonymous=True)
    rospy.loginfo("Path planner node started!")

    reader = MapPoseReader()
    costmap = CostMap(reader=reader)
    nav = Navigator
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
