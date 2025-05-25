#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import socket

class SpeechToCmdVelServer:


    def __init__(self, host="0.0.0.0", port=65434):
        # Networking
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # ROS setup
        rospy.init_node('speech_to_cmd_vel', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(20)

        # Speed parameters
        self.linear_speed = 1.0
        self.angular_speed = 1.0
        self.current_lin = 0.0
        self.current_ang = 0.0
        self.target_lin = 0.0
        self.target_ang = 0.0
        self.SPEED_RAMP = 0.1
        # Flag to control publishing
        self.need_publish = False

    def ramp_speed(self, target, current):
        if abs(target - current) < self.SPEED_RAMP:
            return target
        return current + self.SPEED_RAMP if target > current else current - self.SPEED_RAMP

    def publish_cmd_vel(self, command):
        # adjust speed commands
        if "increase speed" in command or "fast" in command:
            self.linear_speed = min(self.linear_speed + 0.1, 1.0)
            print(f"Linear speed set to: {self.linear_speed}")
        elif "decrease speed" in command or "slow" in command:
            self.linear_speed = max(self.linear_speed - 0.1, 0.0)
            print(f"Linear speed set to: {self.linear_speed}")

        # directional commands set targets
        elif "forward" in command:
            self.target_lin = self.linear_speed
            self.target_ang = 0.0
        elif "backward" in command or "backwards" in command:
            self.target_lin = -self.linear_speed
            self.target_ang = 0.0
        elif "left" in command:
            self.target_lin = 0.0
            self.target_ang = self.angular_speed
        elif "right" in command:
            self.target_lin = 0.0
            self.target_ang = -self.angular_speed
        elif "stop" in command:
            self.target_lin = 0.0
            self.target_ang = 0.0
        else:
            # unknown commands do nothing
            return

        # mark that we need to publish until target is reached
        self.need_publish = True

    def run(self):
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)
        print("Waiting for connection...")
        conn, addr = self.sock.accept()
        conn.setblocking(False)
        print(f"Connected by {addr}")

        try:
            while not rospy.is_shutdown():
                # Non-blocking receive
                try:
                    data = conn.recv(1024)
                    if data:
                        command = data.decode().strip().lower()
                        print(f"Received: {command}")
                        self.publish_cmd_vel(command)
                except BlockingIOError:
                    pass

                # If no need to publish and no motion, skip
                if not self.need_publish:
                    self.rate.sleep()
                    continue

                # Ramping
                self.current_lin = self.ramp_speed(self.target_lin, self.current_lin)
                self.current_ang = self.target_ang  # no angular ramp

                # Publish
                twist = Twist()
                twist.linear.x = self.current_lin
                twist.angular.z = self.current_ang
                self.pub.publish(twist)
                rospy.loginfo(f"Published: linear.x={self.current_lin:.2f}, angular.z={self.current_ang:.2f}")

                # Check if target reached
                if self.current_lin == self.target_lin and self.current_ang == self.target_ang:
                    self.need_publish = False

                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass
        finally:
            conn.close()
            self.sock.close()
            print("[INFO] Socket closed.")

if __name__ == "__main__":
    server = SpeechToCmdVelServer(host="0.0.0.0", port=65434)
    server.run()
