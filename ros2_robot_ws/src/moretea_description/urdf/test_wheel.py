#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class TestWheel(Node):
    def __init__(self):
        super().__init__('test_wheel')
        self.pub = self.create_publisher(Float64MultiArray, "/simple_velocity_controller/commands", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Testing: ONLY Front Left Wheel should spin!")

    def timer_callback(self):
        msg = Float64MultiArray()
        # [FL, RL, FR, RR] -> Sending 5.0 to FIRST item only
        msg.data = [5.0, 0.0, 0.0, 0.0] 
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TestWheel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()