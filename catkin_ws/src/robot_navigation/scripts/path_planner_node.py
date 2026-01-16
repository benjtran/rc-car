#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

def main():
    rospy.init_node('path_planner', anonymous=True)
    rospy.loginfo("Path planner node started!")

    pub = rospy.Publisher('/vel_cmd', Float32, queue_size=10)
    rate = rospy.Rate(4)

    vel = 0.0

    while not rospy.is_shutdown():
        vel = 8 if vel == 0.0 else 0.0
        msg = Float32(data=vel)

        pub.publish(msg)
        rospy.loginfo("Published velocity: %.2f", vel)

        rate.sleep()

if __name__ == "__main__":
    main()
