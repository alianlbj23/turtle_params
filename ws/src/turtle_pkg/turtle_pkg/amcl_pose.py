#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class AMCLTestSubscriber(Node):
    def __init__(self):
        super().__init__('amcl_test_subscriber')

        # 訂閱 /amcl_pose
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.callback,
            10
        )

        self.get_logger().info("已啟動 AMCL 測試訂閱器，等待 /amcl_pose 訊息...")

    def callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = 0.0  # 如果你要 yaw 我可以幫你加 quaternion → yaw

        self.get_logger().info(f"[AMCL] x={x:.3f}, y={y:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = AMCLTestSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
