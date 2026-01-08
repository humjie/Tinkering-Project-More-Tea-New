#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np


class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        
        self.declare_parameter('wheel_radius', 0.029315)
        self.declare_parameter('wheel_separation_width', 0.16)  # distance between left and right wheels
        self.declare_parameter('wheel_separation_length', 0.21)  # distance between front and back wheels

        self.wheel_radius_ = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation_width_ = self.get_parameter('wheel_separation_width').get_parameter_value().double_value
        self.wheel_separation_length_ = self.get_parameter('wheel_separation_length').get_parameter_value().double_value

        self.get_logger().info(f'Using wheel_radius: {self.wheel_radius_}')
        self.get_logger().info(f'Using wheel_separation_width: {self.wheel_separation_width_}')
        self.get_logger().info(f'Using wheel_separation_length: {self.wheel_separation_length_}')

        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "moretea_controller/cmd_vel", self.velCallback, 10)

        # GEOMETRY CALCULATION
        # k = (separation_width + separation_length) / 2.0
        # Sometimes defined strictly as l_x + l_y depending on frame definitions
        self.geom_factor_ = (self.wheel_separation_width_ + self.wheel_separation_length_) / 2.0


    def velCallback(self, msg):
# 1. Extract Robot Velocities (Now includes Linear Y for strafing)
        Vx = msg.twist.linear.x
        Vy = msg.twist.linear.y  # Mecanum can move sideways
        Wz = msg.twist.angular.z

        # 2. Mecanum Inverse Kinematics Equation
        # Calculate individual wheel linear velocities
        # Standard Configuration (O-shape rollers from top view)
        # FL = Front Left, FR = Front Right, RL = Rear Left, RR = Rear Right
        
        # v_fl = Vx - Vy - (lx + ly) * Wz
        # v_fr = Vx + Vy + (lx + ly) * Wz
        # v_rl = Vx + Vy - (lx + ly) * Wz
        # v_rr = Vx - Vy + (lx + ly) * Wz
        
        # Convert to Angular Velocity: omega = v / radius
        
        fl_speed = (Vx - Vy - (self.geom_factor_ * Wz)) / self.wheel_radius_
        fr_speed = (Vx + Vy + (self.geom_factor_ * Wz)) / self.wheel_radius_
        rl_speed = (Vx + Vy - (self.geom_factor_ * Wz)) / self.wheel_radius_
        rr_speed = (Vx - Vy + (self.geom_factor_ * Wz)) / self.wheel_radius_

        # 3. Publish
        # IMPORTANT: check the order of joints in your YAML file! 
        # Usually: [FL, FR, RL, RR] or [FL, RL, FR, RR]
        wheel_speed_msg = Float64MultiArray()
        
        # Assuming Order: Front-Left, Rear-Left, Front-Right, Rear-Right
        wheel_speed_msg.data = [fl_speed, rl_speed, fr_speed, rr_speed]

        self.wheel_cmd_pub_.publish(wheel_speed_msg)


def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()