#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import socket
import sys
import os
'''
Gesture Teleoperation
 >> Takes info from wingesture gesture recognizer script
 >> Converts to twist messages, and sends to /cmd_vel topic
'''
CAMERA_SCRIPT = ""

class GestureToCmdVelServer:
    def __init__(self, host="0.0.0.0", port=65434):
        # Configuration
        self.HOST = host
        self.PORT = port
        self.linear_speed = 1.0
        self.angular_speed = 1.0
        self.current_lin = 0.0
        self.current_ang = 0.0
        self.target_lin = 0.0
        self.target_ang = 0.0
        self.SPEED_RAMP = 0.1

        # ROS setup
        rospy.init_node('gesture_to_cmd_vel', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def print_controls(self):
        print("\n\n")
        print("GESTURE TELEOPERATION")
        print("----------------------------------")
        print("finger count | command")
        print("      0      | STOP")
        print("      1      | d")
        print("      2      | s")
        print("      3      | a")
        print("      4      | w")

        print("  Ctrl+C : Exit")
        print("----------------------------------")
        print(f"Initial linear speed: {self.linear_speed:.2f}")
        print(f"Initial angular speed: {self.angular_speed:.2f}")

    def ramp_speed(self, target, current):
        if abs(target - current) < self.SPEED_RAMP:
            return target
        return current + self.SPEED_RAMP if target > current else current - self.SPEED_RAMP

    def handle_command(self, command):
        if "w" in command:
            self.target_lin = self.linear_speed
            self.target_ang = 0.0
        elif "s" in command and not "stop" in command:
            self.target_lin = -self.linear_speed
            self.target_ang = 0.0
        elif "a" in command:
            self.target_lin = 0.0
            self.target_ang = self.angular_speed
        elif "d" in command:
            self.target_lin = 0.0
            self.target_ang = -self.angular_speed
        else:
            self.target_lin = 0.0
            self.target_ang = 0.0

    def clear_line(self):
        sys.stdout.write('\r' + ' ' * 80 + '\r')
        sys.stdout.flush()

    def show_status(self, command, lin, ang):
        self.clear_line()
        sys.stdout.write(f"[CMD] {command.ljust(15)} | linear.x={lin:.2f} angular.z={ang:.2f}")
        sys.stdout.flush()

    def start(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.HOST, self.PORT))
            s.listen(1)
            os.system('clear')
            print("Waiting for connection...")
            conn, addr = s.accept()
            with conn:
                print(f"Connected by {addr}")
                self.print_controls()
                rate = rospy.Rate(20)  # 20 Hz
                while not rospy.is_shutdown():
                    data = conn.recv(1024)
                    if not data:
                        break
                    command = data.decode().strip()
                    self.handle_command(command)

                    twist = Twist()
                    self.current_lin = self.ramp_speed(self.target_lin, self.current_lin)
                    twist.linear.x = self.current_lin
                    twist.angular.z = self.target_ang
                    self.pub.publish(twist)

                    self.show_status(command, twist.linear.x, twist.angular.z)
                    rate.sleep()

if __name__ == '__main__':
    try:
        server = GestureToCmdVelServer()
        server.start()
    except rospy.ROSInterruptException:
        pass

