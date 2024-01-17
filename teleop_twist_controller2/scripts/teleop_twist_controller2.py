#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import time

info_msg = "\033[32mFor further information, please consult: https://github.com/wenjiu2001/Teleop-Twist-Controller2\033[0m"
warning_msg = "\033[33mThe device number you have set as %d will be automatically reassigned to 1\033[0m"
err_msg = "\033[31mJoystick number %d has not been detected; the current number of device is %d\033[0m\n\033[31mFor further information, please consult: https://github.com/wenjiu2001/Teleop-Twist-Controller2\033[0m"
stop_msg = "\033[32mUse the joystick to control the robot:\033[0m\n\033[32mpush forward or backward for movement, and move left or right for turning.\033[0m"

class TeleopTwistController(Node):
    def __init__(self):
        super().__init__('teleop_twist_controller2')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('device_number', 1),
                ('speed', 0.26),
                ('turn', 1.82),
                ('cmd_vel_topic', '/cmd_vel'),
                ('repeat_rate', 0.0)
            ])

        self.device_number = self.get_parameter('device_number').value
        self.speed = self.get_parameter('speed').value
        self.turn = self.get_parameter('turn').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.repeat_rate = self.get_parameter('repeat_rate').value

        self.check_interval = 0.01
        self.stop_message_interval = 5.0
        self.last_check_time = time.time()
        self.last_stop_time = None
        self.last_x = None
        self.last_th = None
        self.stopped = False

        self.init_joystick()
        self.pub_thread = PublishThread(self, self.cmd_vel_topic, self.repeat_rate)
        self.pub_thread.start()

    def init_joystick(self):
        pygame.init()
        pygame.joystick.init()
        count = pygame.joystick.get_count()
        if self.device_number <= count:
            if self.device_number > 0:
                self.device_number -= 1
            else:
                print(warning_msg % (self.device_number))
                self.device_number = 0
            self.joystick = pygame.joystick.Joystick(self.device_number)
            self.joystick.init()
            print(info_msg)
        else:
            print(err_msg % (self.device_number,count))
            rclpy.shutdown()

    def get_joystick_input(self):
        pygame.event.pump()
        
        x = round(self.joystick.get_axis(1),2)
        th = round(self.joystick.get_axis(0),2)
        
        current_time = time.time()

        if current_time - self.last_check_time >= self.check_interval:
            x = self.joystick.get_axis(1)
            th = self.joystick.get_axis(0)
            
            if x == self.last_x and th == self.last_th:
                if not self.stopped:
                    if self.last_stop_time is None or (current_time - self.last_stop_time >= self.stop_message_interval):
                        self.stopped = True
                        self.last_stop_time = current_time
                        print(stop_msg)
            else:
                self.stopped = False
                self.last_x = x
                self.last_th = th
                print("currently:\tspeed %.2f\tturn %.2f"% (x,th))

            self.last_check_time = current_time

        return x, th

class PublishThread(threading.Thread):
    def __init__(self, node, topic, rate):
        super(PublishThread, self).__init__()
        self.node = node
        self.publisher = node.create_publisher(Twist, topic, 10)
        self.rate = rate
        self.done = False

    def run(self):
        while rclpy.ok() and not self.done:
            x, th = self.node.get_joystick_input()
            twist = Twist()
            twist.linear.x = -x * self.node.speed
            twist.angular.z = -th * self.node.turn
            self.publisher.publish(twist)

    def stop(self):
        self.done = True

def main(args=None):
    rclpy.init(args=args)
    teleop_twist_controller2 = TeleopTwistController()
    try:
        rclpy.spin(teleop_twist_controller2)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_twist_controller2.pub_thread.stop()
        teleop_twist_controller2.pub_thread.join()
        teleop_twist_controller2.destroy_node()
        pygame.quit()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
