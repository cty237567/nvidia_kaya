#!/usr/bin/env python3
import os
import sys
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from dynamixel_sdk import *

ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_MV_Speed = 32
ADDR_MX_CW_EN = 6
ADDR_MX_CCW_EN = 8
ADDR_MX_PRES_POS = 36

PROTOCOL_VERSION = 1

DEVICENAME = "/dev/ttyUSB0"
BAUDRATE = 57600

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.initialize_motor()

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.get_logger().info('Motor control node initialized and waiting for cmd_vel messages.')

    def initialize_motor(self):
        if portHandler.openPort():
            self.get_logger().info("Succeeded to open the port!")
        else:
            self.get_logger().error("Failed to open the port!")
            sys.exit(1)

        if portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info("Succeeded to set the baud rate to {} bps!".format(BAUDRATE))
        else:
            self.get_logger().error("Failed to change the baud rate!")
            sys.exit(1)

        for i in range(3):
            packetHandler.write2ByteTxRx(portHandler, i, ADDR_MX_CW_EN, 0)
            packetHandler.write2ByteTxRx(portHandler, i, ADDR_MX_CCW_EN, 0)

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z
        self.set_vel(vx, vy, omega)
        pos1, pos2, pos3 = self.get_ticks()
        self.publish_odometry(pos1, pos2, pos3)

    def set_vel(self, vx, vy, omega):
        wheel_radius, base_radius = 0.04, 0.1

        v1 = vx - (math.sqrt(3) / 2) * vy + (base_radius * omega)
        v2 = vx + (math.sqrt(3) / 2) * vy + (base_radius * omega)
        v3 = vy - (2 * base_radius * omega)

        omega1 = v1 / wheel_radius
        omega2 = v2 / wheel_radius
        omega3 = v3 / wheel_radius

        omega1 = round((omega1 / (2 * math.pi)) * 60)
        omega2 = round((omega2 / (2 * math.pi)) * (-60))
        omega3 = round((omega3 / (2 * math.pi)) * 60)

        if omega1 < 0:
            omega1 = 1023 + abs(omega1)
        if omega2 < 0:
            omega2 = 1023 + abs(omega2)
        if omega3 < 0:
            omega3 = 1023 + abs(omega3)

        self.get_logger().info(f"Setting motor velocities: {omega1}, {omega2}, {omega3}")
        self.SetMotorVelocities(omega1, omega2, omega3)

    def SetMotorVelocities(self, mot1, mot2, mot3):
        packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_MV_Speed, mot1)
        packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MX_MV_Speed, mot2)
        packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MX_MV_Speed, mot3)

    def get_ticks(self):
        pos1, _, _ = packetHandler.read2ByteTxRx(portHandler, 1, ADDR_MX_PRES_POS)
        pos2, _, _ = packetHandler.read2ByteTxRx(portHandler, 3, ADDR_MX_PRES_POS)
        pos3, _, _ = packetHandler.read2ByteTxRx(portHandler, 2, ADDR_MX_PRES_POS)
        return pos1, pos2, pos3

    def publish_odometry(self, pos1, pos2, pos3):
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = (pos1 + pos2 + pos3) / 3.0
        odom_msg.pose.pose.orientation.w = 1.0  # Assuming no rotation for simplicity

        self.odom_pub.publish(odom_msg)

    def close_motor(self):
        self.SetMotorVelocities(0, 0, 0)
        portHandler.closePort()
        self.get_logger().info("Motors stopped, port closed.")


def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()

    try:
        rclpy.spin(motor_control_node)
    except KeyboardInterrupt:
        pass

    motor_control_node.close_motor()
    motor_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
