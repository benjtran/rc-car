#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Float32MultiArray

PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

ser = None 
last_cmd_time = None
WATCHDOG_TIMEOUT = 1.0  # Stop motors if no command for 1 second

def callback(msg):
    global ser, last_cmd_time
    
    if ser is None or not ser.is_open:
        rospy.logwarn_throttle(5, "Serial port not available")
        return
    
    if len(msg.data) != 4:
        rospy.logwarn("Expected 4 wheel velocities, got %d", len(msg.data))
        return
    
    last_cmd_time = rospy.Time.now()

    clamped = [max(-0.3, min(0.3, v)) for v in msg.data]
    
    # Format: v1,v2,v3,v4\n
    cmd = "{:.3f},{:.3f},{:.3f},{:.3f}".format(*clamped)
    
    try:
        ser.write((cmd + "\n").encode())
        rospy.loginfo_throttle(1, "Sent: %s", cmd)
    except serial.SerialException as e:
        rospy.logerr("Serial write error: %s", e)
        # Try to reconnect
        try:
            ser.close()
            ser.open()
            rospy.logwarn("Serial reconnected")
        except:
            pass

def watchdog_timer(event):
    global ser, last_cmd_time
    
    if last_cmd_time is None:
        return
    
    if ser is None or not ser.is_open:
        return
    
    time_since_last = (rospy.Time.now() - last_cmd_time).to_sec()
    
    if time_since_last > WATCHDOG_TIMEOUT:
        rospy.logwarn_throttle(5, "Watchdog: No commands for %.1fs, stopping motors", time_since_last)
        try:
            stop_cmd = "0.000,0.000,0.000,0.000\n"
            ser.write(stop_cmd.encode())
        except serial.SerialException as e:
            rospy.logerr("Watchdog serial error: %s", e)

def main():
    global ser, last_cmd_time
    
    rospy.init_node('motor_interface', anonymous=True)
    rospy.loginfo("Motor interface node started!")
    
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        rospy.sleep(2.5)  # Allow Arduino to reset (2-3 seconds recommended)
        rospy.loginfo("Serial port opened: %s at %d baud", PORT, BAUD_RATE)
        
        ser.write("0.000,0.000,0.000,0.000\n".encode())
        rospy.loginfo("Sent initial stop command")
        
    except serial.SerialException as e:
        rospy.logerr("Failed to open serial port %s: %s", PORT, e)
        return
    
    rospy.Subscriber('/vel_cmd', Float32MultiArray, callback, queue_size=1)
    
    # Watchdog timer: stops motors if no command received
    rospy.Timer(rospy.Duration(0.5), watchdog_timer)
    
    rospy.loginfo("Listening for velocity commands on /vel_cmd...")
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if ser is not None and ser.is_open:
            rospy.loginfo("Shutting down - stopping motors")
            try:
                ser.write("0.000,0.000,0.000,0.000\n".encode())
                rospy.sleep(0.1)
            except:
                pass
            ser.close()
            rospy.loginfo("Serial port closed")

if __name__ == "__main__":
    main()