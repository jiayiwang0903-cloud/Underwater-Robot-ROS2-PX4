#!/usr/bin/env python3
"""USTROV 控制程序入口。"""

import rclpy
from controller_node import USTROVDirectController


def main():
    rclpy.init()
    node = USTROVDirectController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
