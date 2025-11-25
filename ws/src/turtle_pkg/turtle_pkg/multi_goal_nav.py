#!/usr/bin/env python3

import math
import threading
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class MultiGoalNavigator(Node):
    def __init__(self):
        super().__init__('multi_goal_navigator')

        # === 目標點列表 (x, y, yaw_rad) ===
        self.goals: List[Tuple[float, float, float]] = [
            (-0.10703964, 0.87723856, 0.0),
            (-0.62252312, 1.43241716, 0.0),
            (0.39892093, 1.93724039, 0.0),
        ]

        self.reach_threshold = 0.5
        self.current_goal_idx = 0
        self.current_goal_active = False

        # AMCL 資料
        self.robot_x = None
        self.robot_y = None
        self.have_amcl = False  # ★ 只要收到過 AMCL 就設成 True

        # 尚未按下 Enter 前不開始導航
        self.start_navigation = False

        # === Publisher / Subscriber ===
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # Timer（每秒檢查一次）
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('等待按鍵啟動導航...（按 Enter 開始）')

        # 啟動鍵盤監聽 Thread
        threading.Thread(target=self.wait_for_key, daemon=True).start()

    # ========= 等待按下 Enter ==========
    def wait_for_key(self):
        input("請按 Enter 開始導航流程...\n")
        self.start_navigation = True
        self.get_logger().info("🚀 導航開始！即將送出第一個目標點")

    # ========= Timer 回調 ==========
    def timer_callback(self):
        if not self.start_navigation:
            return

        if not self.have_amcl:
            self.get_logger().warn("尚未取得 AMCL 位置，等待中...")
            return

        if not self.current_goal_active and self.current_goal_idx < len(self.goals):
            self.send_current_goal()

    # ========= Pose 回調 ==========
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.have_amcl = True  # ★ 收到 AMCL → 設 True

        # Debug（你要可改成 info）
        self.get_logger().debug(f"[AMCL] x={self.robot_x:.2f}, y={self.robot_y:.2f}")

        if not self.start_navigation or not self.current_goal_active:
            return

        # 計算是否到達目標
        gx, gy, _ = self.goals[self.current_goal_idx]
        dist = math.hypot(gx - self.robot_x, gy - self.robot_y)

        if dist <= self.reach_threshold:
            self.get_logger().info(
                f"🎯 已到達第 {self.current_goal_idx + 1} 個目標（距離={dist:.2f}m）"
            )

            self.current_goal_active = False
            self.current_goal_idx += 1

            if self.current_goal_idx >= len(self.goals):
                self.get_logger().info("🎉 所有目標皆達成，任務完成！")
                rclpy.shutdown()
            else:
                self.send_current_goal()

    # ========= 發布目標點 ==========
    def send_current_goal(self):
        gx, gy, yaw = self.goals[self.current_goal_idx]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.position.z = 0.0

        # yaw → quaternion
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_pub.publish(msg)
        self.current_goal_active = True

        self.get_logger().info(
            f"🚀 已送出目標 {self.current_goal_idx + 1}/{len(self.goals)}：({gx:.2f}, {gy:.2f})"
        )


# ========= Main ==========
def main(args=None):
    rclpy.init(args=args)
    node = MultiGoalNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
