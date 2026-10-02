#!/usr/bin/env python3

import select
import sys
import termios
import tty

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped



class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('goal_pose_publisher active (press q to quit)')

        self.key = ''
        self.new_input = False
        self.goal_poses = {   # x, y, z, r, p, y
            '0' : [0.8, 0.0, 1.2, np.pi, 0.0, 0.0],
            '1' : [0.6, 0.0, 1.8, np.pi, -np.pi/2, 0.0],
            '2' : [0.0, 0.6, 1.4, np.pi, 0.0, np.pi/2],
            '3' : [-0.6, -0.35, 1.2, np.pi, 0.0, -np.pi/2],
            '4' : [0.1, 0.1, 1.7, np.pi/2, 0.0, 0.0],
            '5' : [0.7, 0.0, 1.0, np.pi, 0.0, 0.0],
            '6' : [0.0, 0.6, 1.4, np.pi, 0.0, -np.pi/2],
            '7' : [0.6, 0.35, 1.5, np.pi, 0.0, np.pi/3],
            '8' : [0.6, -0.5, 1.3, np.pi, 0.0, -np.pi/4],
            'o' : [0.7, 0.0, 1.5, np.pi, 0.0, 0.0],
            'v' : [0.7, 0.0, 1.6, np.pi, 0.0, -np.pi/2],
        }

    def timer_callback(self):
        key = self.get_key()

        if key == 'q':
            self.get_logger().info("Shutting down goal pose publisher, press Ctrl+C to exit.")
            rclpy.shutdown()
            return

        if key != '':
            if key in self.goal_poses:
                self.key = key             # update active goal pose
                self.new_input = True      # mark that we should log once
            else:
                self.get_logger().warning(f"No goal pose [{key}], try another key.")

        if self.key != '':
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            values = self.goal_poses[self.key]

            msg.pose.position.x  = values[0]
            msg.pose.position.y  = values[1]
            msg.pose.position.z  = values[2]

            # Euler angles to quaternion
            quat = self.euler_angles_to_quaternion(values[3], values[4], values[5])
            msg.pose.orientation.w = quat[0]
            msg.pose.orientation.x = quat[1]
            msg.pose.orientation.y = quat[2]
            msg.pose.orientation.z = quat[3]

            self.publisher_.publish(msg)

            if self.new_input:
                self.get_logger().info(f"Publishing goal pose [{self.key}].")
                self.new_input = False

    # Reads the keyboard input, copied from turtlebot3 teleop_keyboard
    def get_key(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def euler_angles_to_quaternion(self, roll, pitch, yaw):
        """
        Computes quaternion corresponding to Euler angle ZYX convention.
        """
        sr, cr = np.sin(roll / 2), np.cos(roll / 2)
        sp, cp = np.sin(pitch / 2), np.cos(pitch / 2)
        sy, cy = np.sin(yaw / 2), np.cos(yaw / 2)

        quat = np.array([
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        ])

        return quat

    
def main(args=None):
    rclpy.init(args=args)
    goal_pose_pub = GoalPosePublisher()
    rclpy.spin(goal_pose_pub)
    goal_pose_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()