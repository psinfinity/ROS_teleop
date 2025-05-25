#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import pygame
import sys

'''
Keyboard Teleoperation
 >> Picks up keystrokes from pygame UI
 >> sends twist commands to /cmd_vel topic
'''

MAX_SPEED = 1.5
MIN_SPEED = 0.1
SPEED_STEP = 0.1
RAMP_STEP = 0.1 

class Teleop:
    def __init__(self):
        rospy.init_node('wasd_teleop')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        pygame.init()
        self.screen = pygame.display.set_mode((360, 450))
        pygame.display.set_caption("🚀 WASD Teleop Controller")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("Consolas", 36, bold=True)

        self.speed = 0.5
        self.current_lin = 0.0
        self.current_ang = 0.0
        self.last_lin = None
        self.last_ang = None

    def print_controls(self):
        print("\n\n")
        print("KEYBOARD TELEOPERATION")
        print("----------------------------------")
        print("   W   ")
        print(" A   D ")
        print("   S   ")
        print(" +   - ")
        print("  Ctrl+C : Exit")
        print("----------------------------------")
        print(f"Initial linear/angular speed: {self.speed:.2f}")

    def send_cmd(self, lx=0, az=0):
        msg = Twist()
        msg.linear.x = lx
        msg.angular.z = az
        self.pub.publish(msg)

    def draw_button(self, label, x, y, active=False):
        base_color = (30, 30, 30)
        active_color = (0, 255, 255)
        shadow_color = (0, 80, 80)
        border_color = (0, 180, 180)

        rect = pygame.Rect(x, y, 80, 80)

        pygame.draw.rect(self.screen, shadow_color, rect.move(4, 4), border_radius=12)
        pygame.draw.rect(self.screen, active_color if active else base_color,
                         rect, border_radius=12)

        if active:
            pygame.draw.rect(self.screen, border_color, rect, 3, border_radius=12)

        text_surface = self.font.render(label, True, (0, 0, 0) if active else (0, 255, 255))
        self.screen.blit(text_surface, (
            x + (80 - text_surface.get_width()) // 2,
            y + (80 - text_surface.get_height()) // 2
        ))

    def draw_speed_slider(self):
        pygame.draw.rect(self.screen, (80, 80, 80), (60, 300, 240, 10))
        handle_x = int(60 + (self.speed - MIN_SPEED) / (MAX_SPEED - MIN_SPEED) * 240)
        pygame.draw.circle(self.screen, (0, 255, 0), (handle_x, 305), 8)
        return pygame.Rect(60, 295, 240, 20)

    def ramp(self, current, target, step):
        if abs(target - current) < step:
            return target
        return current + step if target > current else current - step

    def run(self):
        self.print_controls()
        running = True

        while running and not rospy.is_shutdown():
            self.screen.fill((10, 10, 10))

            keys = pygame.key.get_pressed()
            target_lin = 0
            target_ang = 0
            active = {"W": False, "A": False, "S": False, "D": False}

            if keys[pygame.K_w]:
                target_lin = self.speed
                active["W"] = True
            elif keys[pygame.K_s]:
                target_lin = -self.speed
                active["S"] = True

            if keys[pygame.K_a]:
                target_ang = self.speed
                active["A"] = True
            elif keys[pygame.K_d]:
                target_ang = -self.speed
                active["D"] = True

            # Ramp linear and angular velocities smoothly
            self.current_lin = self.ramp(self.current_lin, target_lin, RAMP_STEP)
            self.current_ang = target_ang

            # Publish only if changed
            if self.current_lin != self.last_lin or self.current_ang != self.last_ang:
                self.send_cmd(self.current_lin, self.current_ang)
                self.last_lin = self.current_lin
                self.last_ang = self.current_ang

            slider_rect = self.draw_speed_slider()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if slider_rect.collidepoint(event.pos):
                        rel_x = event.pos[0] - slider_rect.x
                        new_speed = MIN_SPEED + (rel_x / slider_rect.width) * (MAX_SPEED - MIN_SPEED)
                        self.speed = round(max(MIN_SPEED, min(MAX_SPEED, new_speed)), 2)
                        rospy.loginfo(f"Speed set to: {self.speed:.2f}")
                elif event.type == pygame.KEYDOWN:
                    if event.key in [pygame.K_EQUALS, pygame.K_PLUS]:
                        self.speed = min(MAX_SPEED, self.speed + SPEED_STEP)
                        rospy.loginfo(f"Speed increased → {self.speed:.2f}")
                    elif event.key == pygame.K_MINUS:
                        self.speed = max(MIN_SPEED, self.speed - SPEED_STEP)
                        rospy.loginfo(f"Speed decreased → {self.speed:.2f}")

            self.draw_button("W", 140, 60, active=active["W"])
            self.draw_button("A", 40, 180, active=active["A"])
            self.draw_button("S", 140, 180, active=active["S"])
            self.draw_button("D", 240, 180, active=active["D"])

            pygame.display.flip()
            self.clock.tick(20)

        # Stop robot on exit
        self.send_cmd(0, 0)
        pygame.quit()
        sys.exit()

if __name__ == '__main__':
    try:
        teleop = Teleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
