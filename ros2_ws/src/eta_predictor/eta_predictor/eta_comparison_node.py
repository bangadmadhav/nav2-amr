#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class ETAComparison(Node):

    def __init__(self):
        super().__init__('eta_comparison_node')

        self.predicted = None
        self.actual = None

        self.create_subscription(Float32, '/predicted_time', self.pred_cb, 10)
        self.create_subscription(Float32, '/actual_time', self.actual_cb, 10)

    def pred_cb(self, msg):
        self.predicted = msg.data
        self.try_print()

    def actual_cb(self, msg):
        self.actual = msg.data
        self.try_print()

    def try_print(self):
        if self.predicted is None or self.actual is None:
            return

        error = abs(self.predicted - self.actual)
        percent = (error / self.actual) * 100 if self.actual > 0 else 0

        if percent < 3:
            status = "🟢 EXCELLENT"
        elif percent < 7:
            status = "🟡 GOOD"
        else:
            status = "🔴 NEEDS IMPROVEMENT"
        self.get_logger().info(
            f"\n=== ETA COMPARISON ===\n"
            f"Predicted: {self.predicted:.2f}s\n"
            f"Actual:    {self.actual:.2f}s\n"
            f"Error:     {error:.2f}s ({percent:.2f}%)\n"
            f"Status:    {status}\n"
        )

        # reset for next run
        self.predicted = None
        self.actual = None


def main(args=None):
    rclpy.init(args=args)
    node = ETAComparison()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()