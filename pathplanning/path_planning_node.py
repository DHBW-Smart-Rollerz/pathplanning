# Copyright (c) 2025 Smart Rollerz e.V.
# All rights reserved.

import geometry_msgs.msg
import numpy as np
from camera_preprocessing.transformation.birds_eyed_view import Birdseye
from camera_preprocessing.transformation.coordinate_transform import CoordinateTransform
from camera_preprocessing.transformation.distortion import Distortion
from lane_msgs.msg import Lane, LaneDetectionResult
from smarty_utils.enums import Location, NodeState
from smarty_utils.smarty_node import SmartyNode

from pathplanning.framework.pathplanningController import PPController


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
        "points": [[point.x, point.y, point.z] for point in Lane.points],
        "detected": Lane.detected,
    }


class PathPlanningNode(SmartyNode):
    """ROS Node for path planning."""

    def __init__(self):
        """Initialize the Pathplanning Node."""
        super().__init__(
            "path_planning_node",
            "pathplanning",
            subscribed_topics={
                "/lane_detection/lane": (
                    LaneDetectionResult,
                    self.serialized_points,
                    None,
                )
            },
        )

        self.left_path_publisher = self.create_publisher(
            geometry_msgs.msg.Vector3, "/path_planning/target/left", 10
        )
