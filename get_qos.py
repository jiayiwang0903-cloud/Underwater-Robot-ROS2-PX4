import rclpy
from rclpy.node import Node

class QoSTester(Node):
    def __init__(self):
        super().__init__('qos_tester')
        # We don't even need to subscribe, just spin until we look at tools
        pass

def main():
    rclpy.init()
    node = QoSTester()
    rclpy.spin_once(node, timeout_sec=1.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
