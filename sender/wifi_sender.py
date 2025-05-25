#!/usr/bin/env python3
import socket 
import rospy 
from geometry_msgs.msg import Twist  

'''
Wifi (UDP) Sender
 >> Subscribes to /cmd_vel
 >> Sends data to nodemcu directly via UDP 
'''

class TeleopSender:
    def __init__(self):
        rospy.init_node('teleop_sender')
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.nodemcu_ip = '' 
        self.nodemcu_port = 4210  # Same port as in ESP8266 UDP server

        rospy.Subscriber('/cmd_vel', Twist, self.send_udp_command) # sub to /cmd_vel
        rospy.loginfo("Sender Node Initialized")
    
    def send_udp_command(self, msg): # TODO: optimize UDP packet
        lin = msg.linear.x
        ang = msg.angular.z
        command = "{:.2f},{:.2f}".format(lin, ang)
        self.sock.sendto(command.encode(), (self.nodemcu_ip, self.nodemcu_port))
        rospy.loginfo("Sent: " + command)

if __name__ == '__main__':
    try:
        sender = TeleopSender()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        sender.sock.close()
