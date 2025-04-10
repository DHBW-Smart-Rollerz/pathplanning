# Copyright (c) 2025 Smart Rollerz e.V.
# All rights reserved.

import threading
import time

import cv2
import cv_bridge
import geometry_msgs.msg
import numpy as np
import rclpy
import rclpy.qos
import sensor_msgs.msg
import std_msgs.msg
from ament_index_python.packages import get_package_share_directory
from camera_preprocessing.transformation.birds_eyed_view import Birdseye
from camera_preprocessing.transformation.coordinate_transform import CoordinateTransform
from camera_preprocessing.transformation.distortion import Distortion
from lane_msgs.msg import Lane, LaneDetectionResult
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from smarty_enums.state_machine import GoalLane, NodeState

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


class PathPlanningNode(Node):
    """ROS Node for path planning."""

    def __init__(self):
        """Initialize the Pathplanning Node."""
        super().__init__("pathplanning_node")
        self.package_path = get_package_share_directory("pathplanning")
        self.times = []

        # Load parameters
        self.init_param()

        # Setup Framework & Cord Transformation
        self.cv_bridge = cv_bridge.CvBridge()
        self.coord_trans = CoordinateTransform()
        self.myController = PPController(
            self.get_parameter, debug=self._debug, logger=self.get_logger()
        )
        self.drive_point_ruling = None

        # Initialize transformation classes for debug image
        if self._debug:
            self.distortion = Distortion(self.coord_trans._calib)
            self.birds_eyed = Birdseye(self.coord_trans._calib, self.distortion)

        # Initialize ROS
        self.init_ros()

        # Log initialization
        self.get_logger().info(
            f"Pathplanning Node initialized [debug={self._debug}, trj_look_forward={self.get_parameter('trj_look_forward').value}]"
        )

    @property
    def _debug(self) -> bool:
        return self.get_parameter("debug").value

    @property
    def _state(self) -> NodeState:
        return NodeState(self.get_parameter("state").value)

    @property
    def _goal_lane(self) -> GoalLane:
        return GoalLane(self.get_parameter_or("goal_lane", GoalLane.RIGHT).value)

    def init_param(self):
        """Initialize parameters for the path planning node."""
        # Declare ros parameters
        # Subscribers
        self.declare_parameter("lane_points_subscriber", "/lane_detection/lane")
        self.declare_parameter("image_subscriber", "/camera/image/bev")
        self.declare_parameter("remote_state_subscriber", "/remoteState")

        # Publishers
        self.declare_parameter(
            "targetSteeringAngle_pub", "/control/steering_angle/target"
        )
        self.declare_parameter(
            "path_planning_left_publisher", "/path_planning/target/left"
        )
        self.declare_parameter(
            "path_planning_right_publisher", "/path_planning/target/right"
        )
        self.declare_parameter("ref_point_publisher", "/path_planning/target/pose")
        self.declare_parameter("image_debug_publisher", "/path_planning/debug/image")

        # StateMachine
        self.declare_parameter("trj_look_forward", 100)  # TODO: Remove
        self.declare_parameter("debug", False)
        self.declare_parameter("state", NodeState.INACTIVE.value)
        self.declare_parameter("goal_lane", GoalLane.RIGHT.value)

        # Callbacks
        self.add_on_set_parameters_callback(self.parameter_change_callback)
        self.add_post_set_parameters_callback(self.post_parameter_change_callback)

    def reset(self) -> None:
        """Reset the node."""
        self.get_logger().info("🔄 Resetting the node...")
        self.myController.reset()
        self.set_parameters([Parameter(name="state", value=NodeState.ACTIVE.value)])
        self.get_logger().info("✅ Node reset performed. Continue with ACTIVE state ...")

    def parameter_change_callback(self, params: list[Parameter]) -> SetParametersResult:
        """
        Callback for the parameter server.

        Arguments:
            params -- changed params
        """
        for param in params:
            if self._debug:
                self.get_logger().info(
                    f"Parameter '{param.name}' changed to {param.value}"
                )
        return SetParametersResult(successful=True)

    def post_parameter_change_callback(self, params: list[Parameter]) -> None:
        """
        Post callback for the parameter server.

        Arguments:
            params -- Parameter list
        """
        for param in params:
            if param.name == "state" and param.value == NodeState.RESET.value:
                self.reset()

    def init_ros(self):
        """
        Initializes ROS subscribers and publishers for various topics based on parameters.

        If debug mode is enabled, subscribes to an image topic and sets up a publisher for debug images.
        Subscribes to a state machine topic to set the state.
        Sets up publishers for left and right path planning points, reference points, and lane detection results.
        """
        profile = rclpy.qos.QoSProfile(depth=1)

        ### Debug ###

        if self._debug:
            self.image_subscriber = self.create_subscription(
                sensor_msgs.msg.Image,
                self.get_parameter("image_subscriber").value,
                self.debug_image,
                qos_profile=profile,
            )

            self.image_debug_publisher = self.create_publisher(
                sensor_msgs.msg.Image,
                self.get_parameter("image_debug_publisher").value,
                qos_profile=profile,
            )

        ### Publishers ###

        self.targetSteeringAngle_publisher = self.create_publisher(
            std_msgs.msg.Int16,
            self.get_parameter("targetSteeringAngle_pub").value,
            qos_profile=profile,
        )

        self.path_planning_left_publisher = self.create_publisher(
            geometry_msgs.msg.Vector3,
            self.get_parameter("path_planning_left_publisher").value,
            qos_profile=profile,
        )

        self.path_planning_right_publisher = self.create_publisher(
            geometry_msgs.msg.Vector3,
            self.get_parameter("path_planning_right_publisher").value,
            qos_profile=profile,
        )

        self.ref_point_publisher = self.create_publisher(
            geometry_msgs.msg.Vector3,
            self.get_parameter("ref_point_publisher").value,
            qos_profile=profile,
        )

        ### Subscribers ###

        self.remote_state_subscriber = self.create_subscription(
            std_msgs.msg.UInt8,
            self.get_parameter("remote_state_subscriber").value,
            self.new_remote_state,
            qos_profile=profile,
        )

        self.lane_subscriber = self.create_subscription(
            LaneDetectionResult,
            self.get_parameter("lane_points_subscriber").value,
            self.serialized_points,
            qos_profile=profile,
        )

    def serialized_points(self, LaneDetectionResult: LaneDetectionResult):
        """
        Serialize lane detection results and perform further processing.

        Args:
            LaneDetectionResult (LaneDetectionResult): Detected lane information.
        """
        start_time = time.time()

        self.left_coord = []
        self.center_coord = []
        self.right_coord = []

        if self._state == NodeState.ACTIVE:
            self.serialized_lane_result = {
                "left": serialize_lane(LaneDetectionResult.left),
                "center": serialize_lane(LaneDetectionResult.center),
                "right": serialize_lane(LaneDetectionResult.right),
            }

            for lane_type in ["left", "center", "right"]:
                points = self.serialized_lane_result[lane_type]["points"]
                setattr(self, f"{lane_type}_coord", points)

            (
                self.left_lane_coefficients,
                self.right_lane_coefficients,
            ) = self.myController.start_main_process(
                left_lane_points=self.left_coord,
                center_lane_points=self.center_coord,
                right_lane_points=self.right_coord,
            )

            for lane_type, lane_coefficients in zip(
                ["left", "right"],
                [self.left_lane_coefficients, self.right_lane_coefficients],
            ):
                if self._state == NodeState.ACTIVE and any(lane_coefficients):
                    getattr(self, f"path_planning_{lane_type}_publisher").publish(
                        geometry_msgs.msg.Vector3(
                            x=lane_coefficients[0],
                            y=lane_coefficients[1],
                            z=lane_coefficients[2],
                        )
                    )
            if (
                self._state == NodeState.ACTIVE
                and any(self.left_lane_coefficients)
                and any(self.right_lane_coefficients)
            ):
                self.calculate_ref_point()

            func_time = (
                time.time() - start_time
            ) * 1000  # Calculate the time in milliseconds
            self.times.append(func_time)
            print(f"{sum(self.times) / len(self.times)} ms")
            print("---------------------")

    def calculate_ref_point(self):
        """Calculate the reference point for the vehicle's trajectory based on its current state."""
        if self._state != NodeState.ACTIVE:
            if self._debug:
                self.get_logger().info("Node not active, skipping ...")
            return

        lane_coefficients = (
            self.left_lane_coefficients
            if self._goal_lane == GoalLane.LEFT
            else self.right_lane_coefficients
        )

        if any(lane_coefficients):
            ref_x, ref_y, theta = self.myController.ref_point_controller(
                lane_coefficients
            )
            self.drive_point_ruling = (int(ref_x), int(ref_y))
            print(f"ref_x: {ref_x}, ref_y: {ref_y}, theta: {theta}")
            # ref_x, ref_y, _ = self.coord_trans.bird_to_world([[ref_x, ref_y]])[0]
            print(f"x={ref_x / 1000}, y={ ref_y  / 1000}, theta={theta}")

            if theta <= 0.3:
                theta = theta / 4
                print(theta)

            self.ref_point_publisher.publish(
                geometry_msgs.msg.Vector3(y=ref_y / 1000, x=ref_x / 1000, z=theta)
            )

    def debug_image(self, image_msg: sensor_msgs.msg.Image):
        """
        Processes the image message for debugging purposes and publishes the resulting debug image.

        Args:
            image_msg (sensor_msgs.msg.Image): The image message to be processed.
        """
        # Load parameters (to be persistent)
        is_active = self._state == NodeState.ACTIVE
        debug = self._debug

        if not is_active:
            self.get_logger().info("Node inactive debug image not rendered")
            return

        debug_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="8UC1")
        debug_image = cv2.cvtColor(debug_image, cv2.COLOR_GRAY2RGB)

        if (
            is_active
            and hasattr(self, "serialized_lane_result")
            and self.serialized_lane_result
            and debug
        ):
            for lane_type in ["left", "center", "right"]:
                try:
                    for coord in self.coord_trans.world_to_bird(
                        self.serialized_lane_result[lane_type]["points"]
                    ).astype(int):
                        color = (
                            (255, 0, 0)
                            if lane_type == "left"
                            else (0, 255, 0)
                            if lane_type == "center"
                            else (0, 0, 255)
                        )
                        debug_image = cv2.circle(debug_image, coord, 6, color, -1)
                except Exception as e:
                    self._logger.error(f"Error drawing lane points: {e}")

            for lane_coefficients in [
                self.right_lane_coefficients,
                self.left_lane_coefficients,
            ]:
                for coord in self.myController.draw_trajectory(
                    lane_coefficients, lambda x: self.coord_trans.world_to_bird(x)
                ):
                    cv2.circle(
                        debug_image,
                        (int(coord[0]), int(coord[1])),
                        2,
                        (255, 255, 0),
                        -1,
                    )

            if self.drive_point_ruling is not None:
                p = np.array([[*self.drive_point_ruling, 0]])
                p = self.coord_trans.world_to_bird(p)[0]
                cv2.circle(debug_image, (int(p[0]), int(p[1])), 5, (255, 255, 255), -1)

            self.image_debug_publisher.publish(
                self.cv_bridge.cv2_to_imgmsg(debug_image, encoding="rgb8")
            )

    def new_remote_state(self, remote_state: std_msgs.msg.UInt8):
        """
        Sets the remote state of the system.

        Arguments:
            remote_state -- The new remote state.
        """
        self.myController.reset_RC_MODE(remote_state.data)


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
