#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
import math
import csv
import os
import time


class NavLogger(Node):

    def __init__(self):
        super().__init__('nav_logger_node')

        # ---- FLAGS ----
        self.logging_active = False
        self.pose_ready = False
        self.waiting_for_start = False
        self.pending_plan = None


        self.goal_tolerance = 0.2
        self.create_timer(0.5, self.check_goal_reached)

        # ---- PARAMETERS ----
        self.world = self.declare_parameter('world', 'world1').value
        self.planner = self.declare_parameter('planner', 'navfn').value
        self.controller = self.declare_parameter('controller', 'rpp').value

        # ---- STATE ----
        self.start_pose = None
        self.goal_pose = None
        self.current_pose = None
        self.plan = None

        self.start_time = None
        self.end_time = None
        self.max_run_time = 120  # seconds

        # ---- SUBSCRIBERS ----
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.create_subscription(Path, '/plan', self.plan_callback, 10)

        # ---- CSV ----
        self.csv_file = 'dataset_A.csv'
        self.init_csv()

        self.get_logger().info("Nav Logger Node Started")

    # --------------------------
    def init_csv(self):
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "trial_id", "timestamp", "world", "planner", "controller",
                    "start_x", "start_y", "start_yaw",
                    "goal_x", "goal_y", "goal_yaw",
                    "euclidean_distance",
                    "path_length", "smoothness", "turn_count", "path_efficiency",
                    "ideal_travel_time", "actual_travel_time", "inefficiency_factor",
                    "goal_error_position", "goal_error_yaw",
                    "uncertainty_trace",
                    "navigation_succeeded"
                ])

    # --------------------------
    def pose_callback(self, msg):
        self.current_pose = msg

        cov = msg.pose.covariance

        # ignore fake initial pose (0,0,0 + zero covariance)
        is_fake = (
            msg.pose.pose.position.x == 0.0 and
            msg.pose.pose.position.y == 0.0 and
            cov[0] == 0.0 and cov[7] == 0.0
        )

        if is_fake:
            return

        # valid localization
        is_valid = cov[0] > 1e-4 or cov[7] > 1e-4

        if is_valid:
            if not self.pose_ready:
                self.get_logger().info("AMCL localization is now stable")
            self.pose_ready = True

        # 🔥 START LOGGING HERE
        if self.waiting_for_start and self.pose_ready and not self.logging_active and self.pending_plan is not None:
            self.start_logging(self.pending_plan)
            self.waiting_for_start = False

    # --------------------------
    def plan_callback(self, msg):
        if len(msg.poses) < 2:
            return

        self.plan = msg
        self.pending_plan = msg

        # signal that a goal exists
        if not self.logging_active:
            self.waiting_for_start = True

    # --------------------------
    def start_logging(self, msg):

        self.trial_id = int(time.time() * 1000)
        self.start_pose = self.current_pose
        self.start_time = time.time()

        last_pose = msg.poses[-1].pose.position
        last_orientation = msg.poses[-1].pose.orientation

        self.goal_pose = (
            last_pose.x,
            last_pose.y,
            self.get_yaw(last_orientation)
        )

        self.logging_active = True

        self.get_logger().info(f"Logging started (Trial {self.trial_id})")

    # --------------------------
    def reset(self):
        self.logging_active = False
        self.waiting_for_start = False
        self.pending_plan = None
    
    # --------------------------
    def check_goal_reached(self):

        # 🛑 Guard conditions FIRST
        if not self.logging_active:
            return
        if self.current_pose is None:
            return
        if self.start_time is None:
            return
        if self.goal_pose is None:
            return

        # ✅ Now safe to compute
        elapsed = time.time() - self.start_time

        cx = self.current_pose.pose.pose.position.x
        cy = self.current_pose.pose.pose.position.y

        gx, gy, _ = self.goal_pose

        dist = math.hypot(gx - cx, gy - cy)

        # debug print (throttled)
        if int(time.time()*2) % 4 == 0:
            self.get_logger().info(f"Distance to goal: {dist:.3f}")

        # ✅ SUCCESS
        if dist < 0.3:
            self.end_time = time.time()
            self.compute_and_save(success=True)
            self.get_logger().info("Logging ended: Status -> SUCCESS")
            self.reset()

        # ❌ FAILURE (timeout)
        elif elapsed > self.max_run_time:
            self.end_time = time.time()
            self.compute_and_save(success=False)
            self.get_logger().info("Logging ended: Status -> FAILURE")
            self.reset()

    # --------------------------
    def angle_diff(self, a, b):
        d = a - b
        return math.atan2(math.sin(d), math.cos(d))

    # --------------------------
    def compute_and_save(self, success):

        sx = self.start_pose.pose.pose.position.x
        sy = self.start_pose.pose.pose.position.y
        syaw = self.get_yaw(self.start_pose.pose.pose.orientation)

        gx, gy, gyaw = self.goal_pose

        euclidean_distance = math.hypot(gx - sx, gy - sy)

        path_length, smoothness, turn_count = self.compute_path_features()
        path_efficiency = euclidean_distance / path_length if path_length > 0 else 0

        actual_time = self.end_time - self.start_time
        max_vel = 0.5
        ideal_time = path_length / max_vel if max_vel > 0 else 0
        inefficiency = actual_time / ideal_time if ideal_time > 0 else 0

        cx = self.current_pose.pose.pose.position.x
        cy = self.current_pose.pose.pose.position.y
        cyaw = self.get_yaw(self.current_pose.pose.pose.orientation)

        goal_error_pos = math.hypot(gx - cx, gy - cy)
        goal_error_yaw = abs(self.angle_diff(gyaw, cyaw))

        cov = self.current_pose.pose.covariance
        uncertainty_trace = cov[0] + cov[7] + cov[35]

        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                self.trial_id, time.time(), self.world, self.planner, self.controller,
                sx, sy, syaw,
                gx, gy, gyaw,
                euclidean_distance,
                path_length, smoothness, turn_count, path_efficiency,
                ideal_time, actual_time, inefficiency,
                goal_error_pos, goal_error_yaw,
                uncertainty_trace,
                int(success)
            ])

        self.get_logger().info("Saved dataset row")

    # --------------------------
    def compute_path_features(self):
        if self.plan is None:
            return 0, 0, 0

        poses = self.plan.poses
        length = 0
        smoothness = 0
        turns = 0

        for i in range(1, len(poses)):
            x1 = poses[i-1].pose.position.x
            y1 = poses[i-1].pose.position.y
            x2 = poses[i].pose.position.x
            y2 = poses[i].pose.position.y

            dx = x2 - x1
            dy = y2 - y1

            length += math.hypot(dx, dy)

            if i > 1:
                prev = poses[i-2].pose.position
                angle1 = math.atan2(y1 - prev.y, x1 - prev.x)
                angle2 = math.atan2(dy, dx)
                dtheta = abs(angle2 - angle1)

                smoothness += dtheta
                if dtheta > 0.3:
                    turns += 1

        smoothness = smoothness / (len(poses) - 1) if len(poses) > 0 else 0
        return length, smoothness, turns

    # --------------------------
    def get_yaw(self, q):
        return math.atan2(2*(q.w*q.z), 1 - 2*(q.z*q.z))


def main(args=None):
    rclpy.init(args=args)
    node = NavLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()