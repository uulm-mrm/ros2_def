# pyright: strict

import datetime
from rclpy.executors import Executor


def spin_for(executor: Executor, duration: datetime.timedelta) -> None:
    spin_until(executor, datetime.datetime.now() + duration)


def spin_until(executor: Executor, until: datetime.datetime) -> None:
    while datetime.datetime.now() < until:
        remaining = until - datetime.datetime.now()
        executor.spin_once(timeout_sec=remaining.total_seconds())
