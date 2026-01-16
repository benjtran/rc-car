#!/usr/bin/env python3
import rospy

def main():
    rospy.init_node('path_planner', anonymous=True)
    rospy.loginfo("Path planner node started!")
    rospy.spin()

if __name__ == "__main__":
    main()
