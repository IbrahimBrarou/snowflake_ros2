import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist

import math

import time



class Snowflake(Node):



    def __init__(self):

        super().__init__('snowflake')

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.length = 4.0

        self.angle = 45.0



    def move_turtle_forward(self, length):

        move_cmd = Twist()

        move_cmd.linear.x = length

        self.publisher_.publish(move_cmd)

        time.sleep(1)



    def rotate_turtle(self, angle):

        turn_cmd = Twist()

        turn_cmd.angular.z = math.radians(angle)

        self.publisher_.publish(turn_cmd)

        time.sleep(1)



    def draw_segment(self, length):

        self.move_turtle_forward(length)



    def draw_side_of_snowflake(self, iterations):

        segment_length = self.length / (3 ** iterations)

        for _ in range(iterations):

            self.draw_segment(segment_length)

            self.rotate_turtle(-self.angle)

            self.draw_segment(segment_length)

            self.rotate_turtle(2 * self.angle)

            self.draw_segment(segment_length)

            self.rotate_turtle(-self.angle)

            self.draw_segment(segment_length)



    def draw_full_snowflake(self, iterations):

        self.rotate_turtle(-90)  

        for _ in range(5):

            self.draw_side_of_snowflake(iterations)

            self.rotate_turtle(120)



    def stop_turtle(self):

        self.publisher_.publish(Twist())



def main(args=None):

    rclpy.init(args=args)

    s = Snowflake()

    

    # Set the number of iterations as needed (e.g., 2 in this example)

    iterations = 3

    s.draw_full_snowflake(iterations)

    

    s.stop_turtle()

    s.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':

    main()

