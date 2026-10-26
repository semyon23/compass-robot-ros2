#!/usr/bin/env python3
"""Батарея для симуляции: в Gazebo TB4 нет BMS, а Compass и fms ждут заряд.

Публикует sensor_msgs/BatteryState в battery_state раз в секунду: старт с 80 %, расход
пропорционален пройденному пути (по odom) и немного — стоя. Зарядки нет.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

DRAIN_PER_METER = 0.05    # % на метр: прогон всех тестов (сотни метров) не разряжает до нуля
DRAIN_IDLE_PER_S = 0.002  # % в секунду стоя


class FakeBattery(Node):
    def __init__(self):
        super().__init__('fake_battery')
        self.percent = 80.0
        self.last_xy = None
        self.pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_timer(1.0, self.publish)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.last_xy is not None:
            self.percent -= DRAIN_PER_METER * math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)

    def publish(self):
        self.percent = max(0.0, self.percent - DRAIN_IDLE_PER_S)
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = 24.0
        msg.percentage = self.percent / 100.0  # в BatteryState 0..1
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.present = True
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(FakeBattery())


if __name__ == '__main__':
    main()
