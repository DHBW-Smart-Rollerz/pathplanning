# Copyright (c) 2025 Smart Rollerz e.V.
# All rights reserved.

import numpy as np
import rclpy
import visualization_msgs
from geometry_msgs.msg import Point
from lane_msgs.msg import Lane, LaneDetectionResult
from smarty_utils.smarty_node import SmartyNode
from visualization_msgs.msg import Marker

from pathplanning.algorithms import bspline, huber_regression, ridgecv, theil_sen


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

        self.lane_detection_subscription = self.create_subscription(
            LaneDetectionResult,
            "/lane_detection/lane",
            self.receive_lane_detection_result,
            10,
        )

        self.left_debug_publisher = self.create_publisher(
            visualization_msgs.msg.Marker, "/path_planning/debug/left", 10
        )
        self.center_debug_publisher = self.create_publisher(
            visualization_msgs.msg.Marker, "/path_planning/debug/center", 10
        )
        self.right_debug_publisher = self.create_publisher(
            visualization_msgs.msg.Marker, "/path_planning/debug/right", 10
        )

    def receive_lane_detection_result(self, result: LaneDetectionResult):
        """
        Process serialized lane points.

        Arguments:
            result -- Lane detection result message.
        """
        lanes = [
            ("left", self.left_debug_publisher, (255, 0, 0)),
            ("center", self.center_debug_publisher, (0, 255, 0)),
            ("right", self.right_debug_publisher, (0, 0, 255)),
        ]

        coordinates = {"left": [], "center": [], "right": []}

        for lane_name, publisher, color in lanes:
            lane = getattr(result, lane_name)
            serialized_lane = serialize_lane(lane)

            if len(serialized_lane["points"]) < 10 or not lane.detected:
                continue

            points = ridgecv.ridge(serialized_lane)

            coordinates[lane_name] = points

            self.publish_list_of_points(points, publisher, color)

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
