#!/usr/bin/env python3

"""
Motor Interface node

- Recieves desired velocity from path planner node
- Sends velocity command via serial to motors
"""

import rospy
import serial
from std_msgs.msg import Float32MultiArray

PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

ser = None  # Make global so callback can use it

def callback(msg):
    global ser
    if ser is None:
        return

    if len(msg.data) != 4:
        rospy.logwarn("Expected 4 wheel velocities, got %d", len(msg.data))
        return

    # Format: v1,v2,v3,v4\n
    cmd = "{:.4f},{:.4f},{:.4f},{:.4f}".format(*msg.data)
    ser.write((cmd + "\n").encode())

    rospy.loginfo_throttle(1, "Sent wheel velocities: %s", cmd)

def main():
    global ser
    rospy.init_node('motor_interface', anonymous=True)
    rospy.loginfo("Motor interface node started!")

    # Subscribe to the exact topic
    rospy.Subscriber('/vel_cmd', Float32MultiArray, callback)

    # Open serial port
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    rospy.sleep(2)  # allow Arduino to reset

    rospy.loginfo("Listening for velocity commands...")
    rospy.spin()  # keep node alive, callback handles everything

    ser.close()

if __name__ == "__main__":
    main()


