import datetime
from rclpy.executors import Executor


def spin_for(executor: Executor, duration: datetime.timedelta):
    start = datetime.datetime.now()
    while datetime.datetime.now() - start < duration:
        remaining = datetime.datetime.now() - start
        executor.spin_once(timeout_sec=remaining.total_seconds())
