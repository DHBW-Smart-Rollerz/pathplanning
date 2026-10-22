# Copyright (c) 2025 Smart Rollerz e.V.
# All rights reserved.

import json
import os
import time
from datetime import datetime

import numpy as np
import rclpy
import visualization_msgs
from geometry_msgs.msg import Point
from lane_msgs.msg import Lane, LaneDetectionResult
from smarty_utils.smarty_node import SmartyNode
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker

from pathplanning.ransac import Ransac


def serialize_lane(Lane: Lane):
    """
    Serialize lane information.
    TODO: Should be moved to utils.

    Arguments:
        Lane -- Detected lane information.

    Returns:
        dict -- Serialized lane information.
    """
    return {
        "points": [[point.x / 1000, point.y / 1000] for point in Lane.points],
        "detected": Lane.detected,
    }


class PathPlanningNode(SmartyNode):
    """ROS Node for path planning."""

    def __init__(self):
        """Initialize the Pathplanning Node."""
        super().__init__(
            "path_planning_node",
            "pathplanning",
        )

        self.declare_parameter("crossing_interference", False)

        # Configure logging level based on debug parameter
        if self._debug:
            self._logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # best for seperated fitting
        self.min_x = -0.25
        self.max_x = 1.0

        self.x_vals = np.linspace(
            -1.0, 1.0, num=50
        )  # only used on debug mode to draw trajectory in rviz

        lane_names = ["left", "center", "right"]
        self.lane_filters = {
            name: Ransac(
                self.min_x,
                self.max_x,
                name,
                logger=self._logger,
                buffer_size=10,
                diff_threshold=0.5,
            )
            for name in lane_names
        }

        # not used in caudri challenge. look in readme
        # predefined coeffs for crossing interference (with the direction facing left)
        left_crossing_coeffs = np.array(
            [2.25012465, 3.65954053, 2.53813171, 0.59787335]
        )
        center_crossing_coeffs = np.array(
            [0.52706025, 0.98502997, 1.28259767, 0.59787335]
        )
        right_crossing_coeffs = np.array(
            [-0.30641228, 0.12918641, -0.33166038, 0.59787335]
        )
        self.crossing_coeffs_map = {
            "left": left_crossing_coeffs,
            "center": center_crossing_coeffs,
            "right": right_crossing_coeffs,
        }

        self.lane_detection_subscription = self.create_subscription(
            LaneDetectionResult,
            "/lane_detection/lane",
            self.receive_lane_detection_result,
            10,
        )

        self.crossing_state = 0
        if self.get_parameter("crossing_interference").get_parameter_value().bool_value:
            self.crossing_state_subscription = self.create_subscription(
                String,
                "/state_machine/path_planning/direction",
                self.receive_crossing_state,
                10,
            )
        self.crossing_state_timer = None  # stored to interrupt timer when needed

        # publisher for debug purposes in RViz
        self.left_lane_debug_publisher = self.create_publisher(
            visualization_msgs.msg.Marker, "/path_planning/debug/left", 10
        )
        self.center_lane_debug_publisher = self.create_publisher(
            visualization_msgs.msg.Marker, "/path_planning/debug/center", 10
        )
        self.right_lane_debug_publisher = self.create_publisher(
            visualization_msgs.msg.Marker, "/path_planning/debug/right", 10
        )

        self.left_path_debug_publisher = self.create_publisher(
            visualization_msgs.msg.Marker, "/path_planning/debug/left_path", 10
        )
        self.right_path_debug_publisher = self.create_publisher(
            visualization_msgs.msg.Marker, "/path_planning/debug/right_path", 10
        )

        self.left_path_publisher = self.create_publisher(
            Float32MultiArray, "/path_planning/target/left", 10
        )
        self.right_path_publisher = self.create_publisher(
            Float32MultiArray, "/path_planning/target/right", 10
        )

        self.lanes = [
            ("left", self.left_lane_debug_publisher, (255, 0, 0)),
            ("center", self.center_lane_debug_publisher, (0, 255, 0)),
            ("right", self.right_lane_debug_publisher, (0, 0, 255)),
        ]

        self.timings = []
        # per-frame storage for debugging / analysis
        # Each entry: {"timing": float, "pointcloud": {lane: [[x,y],...]}, "coeffs": {lane: [c0,c1,...]}}
        self.frame_records = []

    def receive_crossing_state(self, msg):
        """
        Receives the crossing state from the state machine.

        Args:
            msg (std_msgs.msg.String): Message containing the crossing state ("left", "right", "none").
        """
        state_str = msg.data.lower()
        if state_str == "straight":
            # start timer, if state was not straight in the last frame to keep this state for a short time
            if self.crossing_state != 0 and self.crossing_state_timer is None:
                self._logger.debug("Starting crossing state timer...")
                self.crossing_state_timer = self.create_timer(
                    4.0, self.reset_crossing_state
                )
            elif self.crossing_state_timer is None:
                # Only set to 0 immediately if there is no active timer
                self.crossing_state = 0
        else:
            # interrupt timer if state changes to crossing again
            if self.crossing_state_timer is not None:
                self.crossing_state_timer.cancel()
                self.crossing_state_timer = None

            if state_str == "left":
                self.crossing_state = 1
            elif state_str == "right":
                self.crossing_state = 1

    def reset_crossing_state(self):
        """Resets the crossing state to 0 after a timer expires."""
        self._logger.debug("Resetting crossing state to 0.")
        self.crossing_state_timer.cancel()
        self.crossing_state_timer = None
        self.crossing_state = 0

    def receive_lane_detection_result(self, result: LaneDetectionResult):
        """
        Process serialized lane points.

        Arguments:
            result -- Lane detection result message.
        """
        self._logger.debug("Received!")

        crossing_state = self.crossing_state

        # serialize lanes now to capture the point cloud as seen by the regressor
        serialized_map = {}
        for lane_name, _, _ in self.lanes:
            lane_msg = getattr(result, lane_name)
            serialized_map[lane_name] = serialize_lane(lane_msg)["points"]

        fit_start = time.time()  # timing to read performance

        coeffs_list = self.fit_all_lanes(result, crossing_state)

        # save timing
        fit_end = time.time()
        frame_timing = fit_end - fit_start
        self.timings.append(frame_timing)

        # store per-frame record (convert numpy arrays to lists for JSON compatibility)
        coeffs_map = {}
        for lane_name in coeffs_list:
            coeffs = coeffs_list[lane_name]
            try:
                coeffs_map[lane_name] = np.array(coeffs).tolist()
            except Exception:
                coeffs_map[lane_name] = []

        self.frame_records.append(
            {
                "timing": frame_timing,
                "pointcloud": serialized_map,
                "coeffs": coeffs_map,
            }
        )

        if self._debug:
            for lane_name, publisher, color in self.lanes:
                coeffs = coeffs_list[lane_name]
                points = self.sample_points_from_poly(coeffs, self.x_vals)
                self.publish_list_of_points(points, publisher, color)

        left_coeffs = []
        right_coeffs = []

        # Calculate midlines between left-center and center-right
        left_coeffs = (coeffs_list["left"] + coeffs_list["center"]) / 2
        if self._debug:
            self.publish_list_of_points(
                self.sample_points_from_poly(left_coeffs, self.x_vals),
                self.left_path_debug_publisher,
                color=(255, 255, 255),
            )

        left_path_msg = Float32MultiArray()
        left_path_msg.data = left_coeffs[::-1].tolist()
        self.left_path_publisher.publish(left_path_msg)

        right_coeffs = (coeffs_list["center"] + coeffs_list["right"]) / 2

        if self._debug:
            self.publish_list_of_points(
                self.sample_points_from_poly(right_coeffs, self.x_vals),
                self.right_path_debug_publisher,
                color=(255, 255, 255),
            )

        right_path_msg = Float32MultiArray()
        right_path_msg.data = right_coeffs[::-1].tolist()
        self.right_path_publisher.publish(right_path_msg)

    def fit_all_lanes(self, lane_result, crossing_state):
        """
        Go through all lanes and fit them.

        Args:
            lane_result (LaneDetectionResult): Result of lane detection topic.
            crossing_state (int): Passed to the lane fitting process to adjust it in case of crossing situations.

        Returns:
            coeffs: Dictionary containing the coefficients for left, center and right lane. Each entry is a list of polynomial coefficients.
        """
        coeffs_list = {"left": [], "center": [], "right": []}

        for lane_name, publisher, color in self.lanes:
            lane = getattr(lane_result, lane_name)
            serialized_lane = serialize_lane(lane)

            if lane.detected:
                coeffs_list[lane_name] = self.lane_filters[lane_name].fit(
                    serialized_lane["points"], crossing_state
                )

        empty_lanes = [
            lane_name for lane_name, coeffs in coeffs_list.items() if len(coeffs) == 0
        ]

        # If all lanes are empty, we need to handle this case separately
        if len(empty_lanes) == 3:
            empty_lanes.clear()

            if crossing_state == 0:
                # using last result from buffer if available, otherwise use straight paths
                self._logger.warning("No lanes found! Using last result.")
                coeffs_list["center"] = (
                    self.lane_filters["center"].buffer[-1]
                    if len(self.lane_filters["center"].buffer) > 0
                    else np.array([0.0, 0.0, 0.0, 0.0])
                )
                coeffs_list["left"] = (
                    self.lane_filters["left"].buffer[-1]
                    if len(self.lane_filters["left"].buffer) > 0
                    else np.array([0.7, 0.0, 0.0, 0.0])
                )
                coeffs_list["right"] = (
                    self.lane_filters["right"].buffer[-1]
                    if len(self.lane_filters["right"].buffer) > 0
                    else np.array([-0.7, 0.0, 0.0, 0.0])
                )
            else:
                self._logger.warning(
                    "No lane pointing to correct crossing direction! Using predefined"
                )
                if crossing_state == -1:
                    coeffs_list = self.crossing_coeffs_map
                else:
                    # swap left and right and invert all coeffs, so lanes face right
                    coeffs_list["left"] = -self.crossing_coeffs_map["right"]
                    coeffs_list["center"] = -self.crossing_coeffs_map["center"]
                    coeffs_list["right"] = -self.crossing_coeffs_map["left"]

                    for lane_name in ["left", "center", "right"]:
                        coeffs_list[lane_name][0] += 0.7

        # Handle cases where some lanes are missing by simulating them based on the detected lanes

        if "left" in empty_lanes:
            self._logger.debug("Left lane missing, simulating...")
            if "center" not in empty_lanes:
                coeffs_list["left"] = coeffs_list["center"].copy()
                coeffs_list["left"][0] += 0.7
            elif "right" not in empty_lanes:
                coeffs_list["left"] = coeffs_list["right"].copy()
                coeffs_list["left"][0] += 1.4
            self.lane_filters["left"].update_buffer(coeffs_list["left"])

        if "right" in empty_lanes:
            self._logger.debug("Right lane missing, simulating...")
            if "center" not in empty_lanes:
                coeffs_list["right"] = coeffs_list["center"].copy()
                coeffs_list["right"][0] -= 0.7
            elif "left" not in empty_lanes:
                coeffs_list["right"] = coeffs_list["left"].copy()
                coeffs_list["right"][0] -= 1.4
            self.lane_filters["right"].update_buffer(coeffs_list["right"])

        if "center" in empty_lanes:
            self._logger.debug("Center lane missing, simulating...")
            if "left" not in empty_lanes and "right" not in empty_lanes:
                coeffs_list["center"] = (coeffs_list["left"] + coeffs_list["right"]) / 2
            elif "left" not in empty_lanes:
                coeffs_list["center"] = coeffs_list["left"].copy()
                coeffs_list["center"][0] -= 0.7
            elif "right" not in empty_lanes:
                coeffs_list["center"] = coeffs_list["right"].copy()
                coeffs_list["center"][0] += 0.7
            self.lane_filters["center"].update_buffer(coeffs_list["center"])

        return coeffs_list

    def sample_points_from_poly(self, coeffs, x_vals):
        """
        Creates sample points along a polynomial curve with given coeffs.

        Args:
            coeffs (array-like): Coefficients of poly.

        Returns:
            list: List of points representing the fitted lane.
        """
        y_vals = np.polyval(
            coeffs[::-1], x_vals
        )  # change coeff order from low-high to high-low
        points = list(map(list, zip(x_vals, y_vals)))

        return points

    def publish_list_of_points(self, points, publisher, color=(1.0, 1.0, 1.0)):
        """
        Publishes a list of points as a line strip marker.

        Arguments:
            points -- List of points to publish. Format: [[x1, y1], [x2, y2], ...].
            publisher -- ROS publisher to use.
            color -- Tuple representing the RGB color of the line (going from 0 - 255).
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lane_polynom"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03  # Line width
        marker.color.a = 1.0
        marker.color.r = color[0] / 255
        marker.color.g = color[1] / 255
        marker.color.b = color[2] / 255
        marker.pose.orientation.w = 1.0

        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0
            marker.points.append(p)
        publisher.publish(marker)

    def empty_marker_topic(self, publisher):
        """
        Publishes an empty marker to clear the topic. Otherwise RViz will keep the last marker on screen.

        Arguments:
            publisher -- ROS publisher to use.
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lane_polynom"
        marker.id = 0
        marker.action = Marker.DELETE
        publisher.publish(marker)

    def log_average_timing(self):
        """Logs the average timing of the fitting process."""
        if self.timings:
            avg_time = sum(self.timings) / len(self.timings)
            self._logger.error(f"Average lane fitting time: {avg_time:.4f} seconds")
            self._logger.error(
                f"Median lane fitting time: {np.median(self.timings):.4f} seconds"
            )

            for lane_filter in self.lane_filters.values():
                self._logger.error(
                    f"Average trials for {lane_filter.lane}: {np.mean(lane_filter.trials):.2f}"
                )
        # If we have per-frame records, save them to a JSON file and clear buffers
        if getattr(self, "frame_records", None):
            try:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

                fname = f"buffer_test_10.json"

                # save to current working directory
                with open(fname, "w", encoding="utf-8") as f:
                    json.dump(self.frame_records, f, ensure_ascii=False, indent=2)
                self._logger.info(
                    f"Saved frame records to {os.path.join('results', os.path.abspath(fname))}"
                )
            except Exception as e:
                self._logger.error(f"Failed to save frame records: {e}")
            finally:
                # clear stored data
                try:
                    self.frame_records.clear()
                except Exception:
                    self.frame_records = []
                try:
                    self.timings.clear()
                except Exception:
                    self.timings = []


def main(args=None):
    """
    Main function to start the path planning node.

    Keyword Arguments:
        args -- Arguments for the Node (default: {None})
    """
    rclpy.init(args=args)
    node = PathPlanningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.log_average_timing()  # comment out if timing not needed
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
