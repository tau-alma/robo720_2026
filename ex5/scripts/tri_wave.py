#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray                               

def tri_wave(t, T):
    # triangle in [-1,1]
    # frac in [0,1): phase
    frac = (t % T) / T
    return 1.0 - 4.0 * abs(frac - 0.5)

class Commander(Node):
    def __init__(self):
        super().__init__('tri_y_commander')
        self.pub = self.create_publisher(Float64MultiArray, 'command', 10)

        # Pose center + orientation (look down)
        self.x  = 0.7
        self.y0 = 0.0
        self.z  = 0.6
        self.r  = math.pi    # roll
        self.p  = 0.0        # pitch
        self.yw = 0.0        # yaw

        self.A  = 0.3        # ±0.3 along Y
        self.T  = 8.0        # seconds per full cycle
        self.t0 = time.time()

        self.timer = self.create_timer(1.0/100.0, self.tick)  # 100 Hz

    def tick(self):
        t = time.time() - self.t0
        y = self.y0 + self.A * tri_wave(t, self.T)
        msg = Float64MultiArray()
        msg.data = [self.x, y, self.z, self.r, self.p, self.yw]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = Commander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()