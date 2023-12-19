import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist

import math

import time



class Snowflake(Node):



    def __init__(self, angle = 45.0):

        super().__init__('snowflake')

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.angle = angle



    def move_turtle_forward(self, length):

        turtle_mov = Twist()

        turtle_mov.linear.x = length

        self.publisher_.publish(turtle_mov)

        time.sleep(1)



    def rotate_turtle(self, angle):

        turtle_turn = Twist()

        turtle_turn.angular.z = math.radians(angle)

        self.publisher_.publish(turtle_turn)

        time.sleep(1)



    def draw_section(self, length):

        self.move_turtle_forward(length)



    def draw_side_of_snowflake(self):

        section_length = 0.3

        for _ in range(2):

            self.draw_section(section_length)

            self.rotate_turtle(-self.angle)

            self.draw_section(section_length)

            self.rotate_turtle(2 * self.angle)

            self.draw_section(section_length)

            self.rotate_turtle(-self.angle)

            self.draw_section(section_length)



    def draw_full_snowflake(self):

        for _ in range(3):

            self.draw_side_of_snowflake()

            self.rotate_turtle(120)



    def stop_turtle(self):

        self.publisher_.publish(Twist())



def main(args=None):

    rclpy.init(args=args)

    s = Snowflake()

    

    s.draw_full_snowflake()

    

    s.stop_turtle()

    s.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':

    main()

