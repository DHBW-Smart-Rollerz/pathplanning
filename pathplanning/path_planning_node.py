# Copyright (c) 2025 Smart Rollerz e.V.
# All rights reserved.

import math

import numpy as np
import rclpy
import visualization_msgs
from geometry_msgs.msg import Point, Vector3
from lane_msgs.msg import Lane, LaneDetectionResult
from smarty_utils.smarty_node import SmartyNode
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker

from pathplanning.algorithms import (
    BSpline,
    HuberRegression,
    KalmanFilter,
    ParticleFilter,
    Ransac,
    RidgeCVRegression,
)

"""
TODO

- simulate/clone lanes when too few points are detected
- include state estimation for accepting turns faster

- improve perfomance (min 30 fps / max 33ms)
"""


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

        self.declare_parameter("crossing_state", 0)

        # Configure logging level based on debug parameter
        if self._debug:
            self._logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.min_x = -1.0
        self.max_x = 1.0

        self.x_vals = np.linspace(self.min_x, self.max_x, num=50)

        lane_names = ["left", "center", "right"]
        self.lane_filters = {
            name: Ransac(
                self.min_x,
                self.max_x,
                name,
                logger=self._logger,
                buffer_size=5,
                diff_threshold=0.5,
            )
            for name in lane_names
        }

        self.lane_detection_subscription = self.create_subscription(
            LaneDetectionResult,
            "/lane_detection/lane",
            self.receive_lane_detection_result,
            10,
        )

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

        self.ref_point_publisher = self.create_publisher(
            Vector3, "/path_planning/target/pose", 10
        )

        self.lanes = [
            ("left", self.left_lane_debug_publisher, (255, 0, 0)),
            ("center", self.center_lane_debug_publisher, (0, 255, 0)),
            ("right", self.right_lane_debug_publisher, (0, 0, 255)),
        ]

    def receive_lane_detection_result(self, result: LaneDetectionResult):
        """
        Process serialized lane points.

        Arguments:
            result -- Lane detection result message.
        """
        self._logger.debug("Received!")

        coeffs_list = {"left": [], "center": [], "right": []}

        crossing_state = (
            self.get_parameter("crossing_state").get_parameter_value().integer_value
        )

        for lane_name, publisher, color in self.lanes:
            lane = getattr(result, lane_name)
            serialized_lane = serialize_lane(lane)

            serialized_lane["points"] = [
                p for p in serialized_lane["points"] if p[0] <= 3
            ]

            if lane.detected:
                coeffs_list[lane_name] = self.lane_filters[lane_name].fit(
                    serialized_lane, crossing_state
                )

        empty_lanes = [
            lane_name for lane_name, coeffs in coeffs_list.items() if len(coeffs) == 0
        ]

        if len(empty_lanes) == 3:
            self._logger.warning("No lanes found! Driving straight")
            empty_lanes.clear()
            coeffs_list["center"] = np.array([0.0, 0.0, 0.0, 0.0])
            coeffs_list["left"] = np.array([0.7, 0.0, 0.0, 0.0])
            coeffs_list["right"] = np.array([-0.7, 0.0, 0.0, 0.0])

        if "left" in empty_lanes:
            self._logger.debug("Left lane missing, simulating...")
            if "center" not in empty_lanes:
                coeffs_list["left"] = coeffs_list["center"].copy()
                coeffs_list["left"][0] += 0.7
            elif "right" not in empty_lanes:
                coeffs_list["left"] = coeffs_list["right"].copy()
                coeffs_list["left"][0] += 1.4

        if "right" in empty_lanes:
            self._logger.debug("Right lane missing, simulating...")
            if "center" not in empty_lanes:
                coeffs_list["right"] = coeffs_list["center"].copy()
                coeffs_list["right"][0] -= 0.7
            elif "left" not in empty_lanes:
                coeffs_list["right"] = coeffs_list["left"].copy()
                coeffs_list["right"][0] -= 1.4

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

        self.calculate_ref_point(right_coeffs)

        if self._debug:
            self.publish_list_of_points(
                self.sample_points_from_poly(right_coeffs, self.x_vals),
                self.right_path_debug_publisher,
                color=(255, 255, 255),
            )

        right_path_msg = Float32MultiArray()
        right_path_msg.data = right_coeffs[::-1].tolist()
        self.right_path_publisher.publish(right_path_msg)

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

    def ref_point_controller(self, coefficients):
        """
        Determines reference points for the controller based on the provided polynomial coefficients.

        Args:
            coefficients (list): List of coefficients representing the polynomial.

        Returns:
            tuple: Tuple containing (x, y, theta) representing the reference point coordinates and angle.
        """
        # self.get_logger().info(
        #     f"Coefficients: {coefficients}, Reversed: {coefficients[::-1]}"
        # )
        p = np.poly1d(coefficients[::-1])
        x = 0.1
        y = p(x)
        y__temp = y
        theta = +1 * math.atan(
            3 * coefficients[3] * ((y__temp) ** 2)
            + 2 * coefficients[2] * (y__temp)
            + coefficients[1]
        )  # Tom fragen
        # self.get_logger().info(f"Ref Point: x: {x}, y: {y}, theta: {theta}")
        return x * 1000, y * 1000, theta

    def calculate_ref_point(self, coeffs):
        """Calculate the reference point for the vehicle's trajectory based on its current state."""
        lane_coefficients = coeffs

        if any(lane_coefficients):
            ref_x, ref_y, theta = self.ref_point_controller(lane_coefficients)
            self.drive_point_ruling = (int(ref_x), int(ref_y))

            if theta <= 0.3:
                theta = theta / 4

            self.ref_point_publisher.publish(
                Vector3(y=ref_y / 1000, x=ref_x / 1000, z=theta)
            )

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
        Publishes an empty marker to clear the topic.

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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
