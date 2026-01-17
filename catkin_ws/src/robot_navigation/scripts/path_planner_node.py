#!/usr/bin/env python3
import rospy
import tf
from std_msgs.msg import Float32

class MapPoseReader:
    def __init__(self):
        self.map = None
        self.map_recieved = False
        self.tf_listener = tf.TransformListener()

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Timer(rospy.Duration(0.5), self.update)
    
    def map_callback(self, msg):
        self.map = msg
        self.map_recieved = True
    
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
        mx, my = self.world_to_map(x, y)

        if mx is None:
            rospy.logwarn("Robot outside map bounds")
            return

        rospy.loginfo(
            "Robot pose: world (%.2f, %.2f) -> map (%d, %d)",
            x, y, mx, my
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

def main():
    rospy.init_node('path_planner', anonymous=True)
    rospy.loginfo("Path planner node started!")

    reader = MapPoseReader()

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
