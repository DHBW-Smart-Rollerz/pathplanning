# Copyright (c) 2025 Smart Rollerz e.V.
# All rights reserved.

import time

import cv_bridge
import geometry_msgs.msg
import numpy as np
import rclpy
import rclpy.clock
import sensor_msgs.msg
import std_msgs.msg
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


class EstimationData:
    """Class to hold estimation data."""

    def __init__(
        self, lane_timestamp: int, lane_points: np.ndarray, world_pose: np.ndarray
    ):
        """
        Initialize the EstimationData class.

        Arguments:
            lane_timestamp -- Timestamp of the lane.
            lane_points -- Points of the lane.
            world_pose -- Pose of the world.
        """
        self.lane_timestamp = lane_timestamp
        self.lane_points = lane_points
        self.world_pose = world_pose

    def __repr__(self):
        """Return a string representation of the EstimationData object."""
        return f"EstimationData(lane_timestamp={self.lane_timestamp}, lane_points={self.lane_points}, world_pose={self.world_pose})"


class PathPlanningNode(SmartyNode):
    """ROS Node for path planning."""

    def __init__(self):
        """Initialize the Pathplanning Node."""
        super().__init__(
            "path_planning_node",
            "pathplanning",
            node_parameters={
                # Subscriber topics
                "lane_points_subscriber": "/lane_detection/lane",
                "image_subscriber": "/camera/image/bev",
                "remote_state_subscriber": "/remoteState",
                "goal_lane_subscriber": "/state_machine/goal_lane",
                # Publisher topics
                "targetSteeringAngle_pub": "/control/steering_angle/target",
                "path_planning_left_publisher": "/path_planning/target/left",
                "path_planning_right_publisher": "/path_planning/target/right",
                "ref_point_publisher": "/path_planning/target/pose",
                "image_debug_publisher": "/path_planning/debug/image",
                # Parameters
                "trj_look_forward": 100,
            },
            subscribed_topics={
                "lane_points_subscriber": (
                    LaneDetectionResult,
                    self.serialized_points,
                    None,
                ),
                "pose_estimation_subscriber": (
                    geometry_msgs.msg.Pose,
                    self.pose_callback,
                    None,
                ),
                "new_image_subscriber": (
                    std_msgs.msg.Header,
                    self.timestamp_callback,
                    None,
                ),
                "image_subscriber": (sensor_msgs.msg.Image, self.debug_image, None),
                "remote_state_subscriber": (
                    std_msgs.msg.UInt8,
                    self.new_remote_state,
                    None,
                ),
                "goal_lane_subscriber": (
                    std_msgs.msg.String,
                    lambda msg: setattr(self, "_goal_lane", Location(msg.data)),
                    None,
                ),
            },
            published_topics={
                "targetSteeringAngle_pub": (std_msgs.msg.Int16, None),
                "path_planning_left_publisher": (geometry_msgs.msg.Vector3, None),
                "path_planning_right_publisher": (geometry_msgs.msg.Vector3, None),
                "ref_point_publisher": (geometry_msgs.msg.Vector3, None),
                "image_debug_publisher": (sensor_msgs.msg.Image, None),
            },
        )
        self._goal_lane = Location.RIGHT
        self.times = []

        # Setup Framework & Cord Transformation
        self.cv_bridge = cv_bridge.CvBridge()
        self.coord_trans = CoordinateTransform()
        self.myController = PPController(
            self.get_parameter, debug=self._debug, logger=self.get_logger()
        )
        self.drive_point_ruling = None

        # Estimation results
        self._est_data: dict[str, EstimationData] = {}
        self.abs_vec = np.zeros(3)  # x, y, psi
        self.est_vec = np.zeros(3)  # x, y, psi

        # Initialize transformation classes for debug image
        if self._debug:
            self.distortion = Distortion(self.coord_trans._calib)
            self.birds_eyed = Birdseye(self.coord_trans._calib, self.distortion)

        # Log initialization
        self.get_logger().info(
            f"Path planning Node initialized [debug={self._debug}, trj_look_forward={self.get_parameter('trj_look_forward').value}]"
        )

    def pose_callback(self, pose_msg: geometry_msgs.msg.Pose):
        """
        Callback function for pose estimation.

        Arguments:
            pose_msg -- The pose message.
        """
        self.abs_vec = np.array(
            [
                pose_msg.position.x,
                pose_msg.position.y,
                pose_msg.position.z,
            ]
        )

        self.est_vec = self.abs_vec - self.zero_vec

        # Estimate new lane
        self.estimate_and_publish_lane()

        if self._debug:
            self.get_logger().info(
                f"Pose Estimation: x={self.abs_vec[0]}, y={self.abs_vec[1]}, psi={self.abs_vec[2]}"
            )

    def timestamp_callback(self, msg: std_msgs.msg.Header) -> None:
        """Resets the estimation to zero."""
        ts = f"{msg.stamp.sec}.{msg.stamp.nanosec}"

        if ts in self._est_data:
            rclpy.get_logger().warn(
                f"Timestamp {ts} already exists in _est_data. Overwriting."
            )
        self._est_data[ts] = EstimationData(
            lane_timestamp="",
            lane_points=[],
            world_pose=self.abs_vec,
        )

        self.shrink_estimation_data()

        if self._debug:
            self.get_logger().info(
                f"Timestamp {ts} added to _est_data with world pose: {self.abs_vec}"
            )

    def shrink_estimation_data(self, max_size: int = 3):
        """
        Shrink the estimation data to the specified maximum size.

        Arguments:
            max_size -- The maximum size of the estimation data.
        """
        if len(self._est_data.keys()) > max_size:
            time_stamps = list(self._est_data.keys())
            time_stamps.sort(reverse=True)
            keys_to_remove = time_stamps[: len(self._est_data) - max_size]
            for key in keys_to_remove:
                del self._est_data[key]
                if self._debug:
                    self.get_logger().info(f"Removed timestamp {key} from _est_data")

    def estimate_and_publish_lane(self):
        """Transform the lane points to the world coordinates and publish them."""
        # TODO: Check everything
        # Shape 3 x 3
        T = np.array(
            [
                [np.cos(self.est_vec[2]), -np.sin(self.est_vec[2]), self.est_vec[0]],
                [np.sin(self.est_vec[2]), np.cos(self.est_vec[2]), self.est_vec[1]],
                [0, 0, 1],
            ]
        )
        # Shape 2 x 1 (10 times)
        left_points = np.array(self._est_data["left"].lane_points)
        right_points = np.array(self._est_data["right"].lane_points)

        # Make homogeneous coordinates
        left_points_homogeneous = np.hstack(
            (left_points, np.ones((left_points.shape[0], 1)))
        )
        right_points_homogeneous = np.hstack(
            (right_points, np.ones((right_points.shape[0], 1)))
        )

        # shape 3 x 10
        left_transformed = np.dot(T, left_points_homogeneous.T)
        right_transformed = np.dot(T, right_points_homogeneous.T)

        # shape 2 x 10
        left_transformed = left_transformed[:2, :].T
        right_transformed = right_transformed[:2, :].T

        # Make polyfit
        left_lane_coefficients = np.polyfit(
            left_transformed[:, 0], left_transformed[:, 1], 2
        )
        right_lane_coefficients = np.polyfit(
            right_transformed[:, 0], right_transformed[:, 1], 2
        )

        # Publish the lane coefficients
        for lane_type, lane_coefficients in zip(
            ["left", "right"],
            [left_lane_coefficients, right_lane_coefficients],
        ):
            if self._state == NodeState.ACTIVE and any(lane_coefficients):
                getattr(self, f"path_planning_{lane_type}_publisher").publish(
                    geometry_msgs.msg.Vector3(
                        x=lane_coefficients[0],
                        y=lane_coefficients[1],
                        z=lane_coefficients[2],
                    )
                )

    def _reset(self) -> None:
        """Reset the node."""
        self.myController.reset()

    def serialized_points(self, LaneDetectionResult: LaneDetectionResult):
        """
        Serialize lane detection results and perform further processing.

        Args:
            LaneDetectionResult (LaneDetectionResult): Detected lane information.
        """
        if self._state != NodeState.ACTIVE:
            return

        s, ns = rclpy.clock.Clock().now().seconds_nanoseconds()
        ts = f"{s}.{ns}"

        if ts not in self._est_data.keys():
            rclpy.get_logger().warn(
                f"Timestamp {ts} not found in _est_data. Skipping processing."
            )
            self._est_data[ts] = EstimationData(
                lane_timestamp="",
                lane_points=[],
                world_pose=self.abs_vec,
            )

        self._est_data[ts].lane_timestamp = ts
        start_time = time.time()

        # Get the lane points
        self.left_coord = []
        self.center_coord = []
        self.right_coord = []

        self._est_data[ts].lane_points = ""  # TODO

        # Get the driving lane
        (
            self.left_lane_coefficients,
            self.right_lane_coefficients,
        ) = self.myController.start_main_process(
            left_lane_points=self.serialize_lane(LaneDetectionResult.left),
            center_lane_points=self.serialize_lane(LaneDetectionResult.center),
            right_lane_points=self.serialize_lane(LaneDetectionResult.right),
        )

        # Get 10 points for each lane
        x = np.linspace(-100, 1500, 10)
        y_left = np.polyval(self.left_lane_coefficients, x)
        y_right = np.polyval(self.right_lane_coefficients, x)
        left_points = np.array(list(zip(x, y_left)))
        right_points = np.array(list(zip(x, y_right)))
        self._est_data[ts].lane_points = {"left": left_points, "right": right_points}

        # Publish Lanes
        # for lane_type, lane_coefficients in zip(
        #     ["left", "right"],
        #     [self.left_lane_coefficients, self.right_lane_coefficients],
        # ):
        #     if self._state == NodeState.ACTIVE and any(lane_coefficients):
        #         getattr(self, f"path_planning_{lane_type}_publisher").publish(
        #             geometry_msgs.msg.Vector3(
        #                 x=lane_coefficients[0],
        #                 y=lane_coefficients[1],
        #                 z=lane_coefficients[2],
        #             )
        #         )

        # if (
        #     self._state == NodeState.ACTIVE
        #     and any(self.left_lane_coefficients)
        #     and any(self.right_lane_coefficients)
        # ):
        #     self.calculate_ref_point()

        func_time = (
            time.time() - start_time
        ) * 1000  # Calculate the time in milliseconds
        self.times.append(func_time)
        print(f"{sum(self.times) / len(self.times)} ms")
        print("---------------------")

    # def calculate_ref_point(self):
    #     """Calculate the reference point for the vehicle's trajectory based on its current state."""
    #     if self._state != NodeState.ACTIVE:
    #         if self._debug:
    #             self.get_logger().info("Node not active, skipping ...")
    #         return

    #     lane_coefficients = (
    #         self.left_lane_coefficients
    #         if self._goal_lane == Location.LEFT
    #         else self.right_lane_coefficients
    #     )

    #     if any(lane_coefficients):
    #         ref_x, ref_y, theta = self.myController.ref_point_controller(
    #             lane_coefficients
    #         )
    #         self.drive_point_ruling = (int(ref_x), int(ref_y))
    #         print(f"ref_x: {ref_x}, ref_y: {ref_y}, theta: {theta}")
    #         # ref_x, ref_y, _ = self.coord_trans.bird_to_world([[ref_x, ref_y]])[0]
    #         print(f"x={ref_x / 1000}, y={ ref_y  / 1000}, theta={theta}")

    #         if theta <= 0.3:
    #             theta = theta / 4
    #             print(theta)

    #         self.ref_point_publisher.publish(
    #             geometry_msgs.msg.Vector3(y=ref_y / 1000, x=ref_x / 1000, z=theta)
    #         )

    # def debug_image(self, image_msg: sensor_msgs.msg.Image):
    #     """
    #     Processes the image message for debugging purposes and publishes the resulting debug image.

    #     Args:
    #         image_msg (sensor_msgs.msg.Image): The image message to be processed.
    #     """
    #     # Load parameters (to be persistent)
    #     is_active = self._state == NodeState.ACTIVE
    #     debug = self._debug

    #     if not is_active:
    #         self.get_logger().info("Node inactive debug image not rendered")
    #         return

    #     debug_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="8UC1")
    #     debug_image = cv2.cvtColor(debug_image, cv2.COLOR_GRAY2RGB)

    #     if (
    #         is_active
    #         and hasattr(self, "serialized_lane_result")
    #         and self.serialized_lane_result
    #         and debug
    #     ):
    #         for lane_type in ["left", "center", "right"]:
    #             try:
    #                 for coord in self.coord_trans.world_to_bird(
    #                     self.serialized_lane_result[lane_type]["points"]
    #                 ).astype(int):
    #                     color = (
    #                         (255, 0, 0)
    #                         if lane_type == "left"
    #                         else (0, 255, 0) if lane_type == "center" else (0, 0, 255)
    #                     )
    #                     debug_image = cv2.circle(debug_image, coord, 6, color, -1)
    #             except Exception as e:
    #                 self._logger.error(f"Error drawing lane points: {e}")

    #         for lane_coefficients in [
    #             self.right_lane_coefficients,
    #             self.left_lane_coefficients,
    #         ]:
    #             for coord in self.myController.draw_trajectory(
    #                 lane_coefficients, lambda x: self.coord_trans.world_to_bird(x)
    #             ):
    #                 cv2.circle(
    #                     debug_image,
    #                     (int(coord[0]), int(coord[1])),
    #                     2,
    #                     (255, 255, 0),
    #                     -1,
    #                 )

    #         if self.drive_point_ruling is not None:
    #             p = np.array([[*self.drive_point_ruling, 0]])
    #             p = self.coord_trans.world_to_bird(p)[0]
    #             cv2.circle(debug_image, (int(p[0]), int(p[1])), 5, (255, 255, 255), -1)

    #         self.image_debug_publisher.publish(
    #             self.cv_bridge.cv2_to_imgmsg(debug_image, encoding="rgb8")
    #         )

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
