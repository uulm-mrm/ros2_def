import datetime
import rclpy
from rclpy.node import Node


def spin_for(node: Node, duration: datetime.timedelta):
    start = datetime.datetime.now()
    while datetime.datetime.now() - start < duration:
        remaining = datetime.datetime.now() - start
        rclpy.spin_once(node, timeout_sec=remaining.total_seconds())
