#!/usr/bin/env python3

"""
Motor Interface node

- Recieves desired velocity from path planner node
- Sends velocity command via serial to motors
"""

import rospy
import serial
import time

PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

def send_command(ser, cmd):
    ser.write((cmd + '\n').encode())

def main():
    rospy.init_node('motor_interface', anonymous=True)
    rospy.loginfo("Motor interface node started!")
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        send_command(ser, "24.4")
        time.sleep(2)
        send_command(ser, "0.0")
        time.sleep(2)
    ser.close()

if __name__ == "__main__":
    main()

