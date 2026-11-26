
"""Known-location ArUco mission for Lab 7."""

import math
import time

import rclpy

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.time import Time
from tf2_ros import TransformException

from mission.node import MapPose, MissionNode


class AutonomousSearch(MissionNode):

    def navigate_to(self, pose: MapPose) -> None:

        x_goal, y_goal, yaw_goal = pose

        position_tol = self.position_tolerance_m
        heading_tol = self.heading_tolerance_rad

        # 1. Check the Nav2 action server
        if not self.action.wait_for_server(
            timeout_sec=self.goal_timeout_s
        ):
            raise TimeoutError("Nav2 action server unavailable")

        # 2. Construct the stamped navigation goal
        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(x_goal)
        goal.pose.pose.position.y = float(y_goal)
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw_goal / 2)
        goal.pose.pose.orientation.w = math.cos(yaw_goal / 2)

        self.get_logger().info(
            f"NAV GOAL: x={x_goal:.3f}, "
            f"y={y_goal:.3f}, yaw={yaw_goal:.3f}"
        )

        deadline = time.monotonic() + self.goal_timeout_s

        # 3. Send the navigation goal
        send_future = self.action.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=max(0.0, deadline - time.monotonic()),
        )

        if not send_future.done():
            # Cancel if a delayed acceptance arrives later.
            def cancel_late_acceptance(future):
                try:
                    handle = future.result()
                    if handle is not None and handle.accepted:
                        handle.cancel_goal_async()
                except Exception:
                    pass

            send_future.add_done_callback(cancel_late_acceptance)

            raise TimeoutError("Navigation acceptance timeout")

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Nav2 rejected the goal")

        # 4. Wait for the navigation result
        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=max(0.0, deadline - time.monotonic()),
        )

        if not result_future.done():
            cancel_future = goal_handle.cancel_goal_async()

            rclpy.spin_until_future_complete(
                self,
                cancel_future,
                timeout_sec=2.0,
            )

            raise TimeoutError("Navigation timed out; cancellation requested")

        result = result_future.result()

        if result is None:
            raise RuntimeError("Missing Nav2 result")

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"Navigation failed: status={result.status}"
            )

        self.get_logger().info("NAV2 RESULT: SUCCEEDED")

        # 5. Independently verify the measured TF pose
        arrival_deadline = (
            time.monotonic() + self.arrival_timeout_s
        )

        last_error = "No usable TF transform"

        while rclpy.ok() and time.monotonic() < arrival_deadline:

            try:
                tf = self.tf_buffer.lookup_transform(
                    "map",
                    self.base_frame,
                    Time(),
                )

                translation = tf.transform.translation
                rotation = tf.transform.rotation

                x = translation.x
                y = translation.y

                # Quaternion to yaw
                yaw = math.atan2(
                    2.0 * (
                        rotation.w * rotation.z
                        + rotation.x * rotation.y
                    ),
                    1.0 - 2.0 * (
                        rotation.y ** 2 + rotation.z ** 2
                    ),
                )

                # Position error
                position_error = math.hypot(
                    x - x_goal,
                    y - y_goal,
                )

                # Wrapped heading error
                heading_error = abs(
                    math.remainder(
                        yaw - yaw_goal,
                        2.0 * math.pi,
                    )
                )

                last_error = (
                    f"position_error={position_error:.3f} m, "
                    f"heading_error={heading_error:.3f} rad"
                )

                if (
                    position_error <= position_tol
                    and heading_error <= heading_tol
                ):
                    self.get_logger().info(
                        f"ARRIVAL VERIFIED: "
                        f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}; "
                        f"{last_error}"
                    )
                    return

            except TransformException as error:
                last_error = str(error)

            rclpy.spin_once(
                self,
                timeout_sec=min(
                    0.05,
                    max(
                        0.0,
                        arrival_deadline - time.monotonic(),
                    ),
                ),
            )

        raise RuntimeError(
            f"Arrival verification failed: {last_error}"
        )

    def run_mission(self) -> str:

        readings = {}

        self.get_logger().info(
            f"MISSION START: target={self.target_fruit}"
        )

        # 1. Visit every room in the required order
        for room in self.visit_order:

            self.get_logger().info(
                f"VISITING: {room}"
            )

            self.navigate_to(self.view_poses[room])

            # 2. Read only after verified arrival
            reading = self.marker_reader.read_marker()

            if reading is None:
                raise RuntimeError(
                    f"No fresh marker reading at {room}"
                )

            marker_id, label, stamp_ns = reading

            readings[room] = {
                "id": marker_id,
                "label": label,
                "stamp_ns": stamp_ns,
            }

            self.get_logger().info(
                f"MARKER: room={room}, "
                f"id={marker_id}, label={label}, "
                f"stamp_ns={stamp_ns}"
            )

        # 3. Select a unique match only after all readings
        matches = [
            room
            for room, observation in readings.items()
            if observation["label"] == self.target_fruit
        ]

        if len(matches) != 1:
            raise RuntimeError(
                f"Expected one matching room, "
                f"found {len(matches)}: {matches}"
            )

        selected_room = matches[0]

        self.get_logger().info(
            f"TARGET SELECTED: {selected_room}"
        )

        # 4. Navigate to the selected parking pose
        self.navigate_to(
            self.goal_poses[selected_room]
        )

        self.get_logger().info(
            f"PARKING VERIFIED: {selected_room}"
        )

        return selected_room
