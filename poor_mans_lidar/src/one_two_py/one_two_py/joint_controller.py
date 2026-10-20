#!/usr/bin/env python3 
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Float64

class Joint_Controller_Node(Node):
    def __init__(self):
        super().__init__("joint_controller")
        
        # 1. Subscriber to Ultrasonic LaserScan
        self.subscriber_ = self.create_subscription(
            LaserScan, "sonar/distance", self.callback_function, 10
        )
        
        # 2. Publisher for ESP32 Physical Servo
        self.cmd_publisher_ = self.create_publisher(Float64, "joint_cmd_pos", 10)
        
        # 3. Publisher for RViz2 / robot_state_publisher (TF transform)
        self.joint_publisher_ = self.create_publisher(JointState, "/joint_states", 10)
        
        self.timer_ = self.create_timer(0.05, self.publish_turn)
        self.get_logger().info("The joint controller has started....")
        
        self.current_distance = 0.0
        self.target_angle = 0.0
        self.direction = 1
        self.step_size = 0.035  # ~2 degrees per step

    def publish_turn(self):
        # Update target angle (0 to pi radians / 0 to 180 degrees)
        self.target_angle += self.direction * self.step_size
        max_angle = math.pi  # 180 degrees for MG90S Servo

        if self.target_angle >= max_angle:
            self.target_angle = max_angle
            self.direction = -1
        elif self.target_angle <= 0.0:
            self.target_angle = 0.0
            self.direction = 1

        # A. Publish Float64 to ESP32 servo motor
        cmd = Float64()
        cmd.data = self.target_angle
        self.cmd_publisher_.publish(cmd)

        # B. Publish JointState to drive 3D model in RViz2
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = ['base_sholder_joint']  # Your exact URDF joint name
        js_msg.position = [self.target_angle]
        self.joint_publisher_.publish(js_msg)

    def callback_function(self, msg):
        if msg.ranges:
            self.current_distance = min(msg.ranges)
            self.get_logger().info(
                f'Distance: {self.current_distance:.2f} m | Angle: {math.degrees(self.target_angle):.1f}°'
            )

def main(args=None):
    rclpy.init(args=args)
    node = Joint_Controller_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()