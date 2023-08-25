# pyright: strict
from rclpy.impl.rcutils_logger import RcutilsLogger


def lc(logger: RcutilsLogger, msg: str) -> bool:
    return logger.info('\033[96m' + msg + '\033[0m')
