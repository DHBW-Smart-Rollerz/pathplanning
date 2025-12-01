# Copyright (c) 2025 Smart Rollerz e.V.
# All rights reserved.

import numpy as np
import rclpy
import visualization_msgs
from geometry_msgs.msg import Point
from lane_msgs.msg import Lane, LaneDetectionResult
from smarty_utils.smarty_node import SmartyNode
from visualization_msgs.msg import Marker

from pathplanning.algorithms import (
    BSpline,
    HuberRegression,
    KalmanFilter,
    ParticleFilter,
    RidgeCV,
    RidgeRansac,
)
from pathplanning.lane_filter_manager import LaneFilterManager


class PathPlanningNode(SmartyNode):
    """ROS Node for path planning."""

    def __init__(self):
        """Initialize the Pathplanning Node."""
        super().__init__(
            "path_planning_node",
            "pathplanning",
        )

        lane_names = ["left", "center", "right"]
        self.lane_filters = {name: RidgeRansac() for name in lane_names}

        self.lane_filter_manager = LaneFilterManager(
            self.lane_filters, buffer_size=3, diff_threshold=0.15, logger=self._logger
        )

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
        coordinates = self.lane_filter_manager.fit(result)

        for lane_name, publisher, color in self.lanes:
            points = coordinates[lane_name]
            if points:
                self.publish_list_of_points(points, publisher, color)
            else:
                self.empty_marker_topic(publisher)

        return  # remove this line to enable midline publishing

        # Calculate midlines between left-center and center-right
        if len(coordinates["left"]) > 0 and len(coordinates["center"]) > 0:
            mid_left_center = [
                [(l[0] + c[0]) / 2, (l[1] + c[1]) / 2]
                for l, c in zip(coordinates["left"], coordinates["center"])
            ]
            self.publish_list_of_points(
                mid_left_center, self.left_path_debug_publisher, (255, 255, 255)
            )
        else:
            self.empty_marker_topic(self.left_path_debug_publisher)

        if len(coordinates["center"]) > 0 and len(coordinates["right"]) > 0:
            mid_center_right = [
                [(c[0] + r[0]) / 2, (c[1] + r[1]) / 2]
                for c, r in zip(coordinates["center"], coordinates["right"])
            ]
            self.publish_list_of_points(
                mid_center_right, self.right_path_debug_publisher, (255, 255, 255)
            )
        else:
            self.empty_marker_topic(self.right_path_debug_publisher)

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
