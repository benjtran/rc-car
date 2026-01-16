#!/usr/bin/env python3
import rospy

def main():
    rospy.init_node('motor_interface', anonymous=True)
    rospy.loginfo("Motor interface node started!")
    rospy.spin()

if __name__ == "__main__":
    main()
