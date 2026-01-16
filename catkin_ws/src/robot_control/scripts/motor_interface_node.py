#!/usr/bin/env python3

"""
Motor Interface node

- Recieves desired velocity from path planner node
- Sends velocity command via serial to motors
"""

import rospy
import serial
from std_msgs.msg import Float32

PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

ser = None  # Make global so callback can use it

def callback(msg):
    global ser
    if ser is not None:
        cmd = str(msg.data)
        ser.write((cmd + '\n').encode())
        rospy.loginfo("Sent command to motor: %s", cmd)

def main():
    global ser
    rospy.init_node('motor_interface', anonymous=True)
    rospy.loginfo("Motor interface node started!")

    # Subscribe to the exact topic
    rospy.Subscriber('/vel_cmd', Float32, callback)

    # Open serial port
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    rospy.sleep(2)  # allow Arduino to reset

    rospy.loginfo("Listening for velocity commands...")
    rospy.spin()  # keep node alive, callback handles everything

    ser.close()

if __name__ == "__main__":
    main()


