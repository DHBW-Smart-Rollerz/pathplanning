# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import struct

import numpy as np
import rclpy
from lane_msgs.msg import LaneDetectionResult
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class LanesToPointCloudNode(Node):
    """Node that converts lane detection results PointCloud2."""

    def __init__(self):
        """Initialize the LanesToPointCloudNode, set up subscriptions and publishers for lane point clouds."""
        super().__init__("lanes_to_point_cloud_node")
        self.subscription = self.create_subscription(
            LaneDetectionResult, "/lane_detection/lane", self.listener_callback, 10
        )
        self.left_publisher = self.create_publisher(
            PointCloud2, "/pathplanning/lanes/left", 10
        )
        self.center_publisher = self.create_publisher(
            PointCloud2, "/pathplanning/lanes/center", 10
        )
        self.right_publisher = self.create_publisher(
            PointCloud2, "/pathplanning/lanes/right", 10
        )
        self.subscription  # prevent unused variable warning

    def lane_to_points(self, lane):
        """
        Convert a lane message to a list of (x, y, z) tuples if detected.

        Args:
            lane (lane_msgs.msg.Lane): Lane message containing points.

        Returns:
            list: List of (x, y, z) tuples.
        """
        return [(p.x, p.y, p.z) for p in lane.points] if lane.detected else []

    def listener_callback(self, msg):
        """
        Callback for lane detection results. Publishes each lane as a colored PointCloud2 message.

        Args:
            msg (lane_msgs.msg.LaneDetectionResult): Lane detection result message.
        """
        self.get_logger().info("Lane detected!")

        # change points from millimeter to meter
        for lane in (msg.left, msg.center, msg.right):
            if getattr(lane, "detected", False):
                for p in lane.points:
                    p.x /= 1000.0
                    p.y /= 1000.0
                    p.z /= 1000.0

        lanes = [
            (msg.left, self.left_publisher, (255, 0, 0)),  # Red for left
            (msg.center, self.center_publisher, (0, 255, 0)),  # Green for center
            (msg.right, self.right_publisher, (0, 0, 255)),  # Blue for right
        ]

        for lane, publisher, color in lanes:
            points = self.lane_to_points(lane)
            point_count = len(points)
            if point_count == 0:
                continue

            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
            ]

            # some shenanigans copilot did so the color works
            rgb = (color[0] << 16) | (color[1] << 8) | color[2]
            rgb_float = struct.unpack("f", struct.pack("I", rgb))[0]

            points_array = np.zeros((point_count, 4), dtype=np.float32)
            points_array[:, 0:3] = np.array(points, dtype=np.float32)
            points_array[:, 3] = rgb_float
            data = points_array.tobytes()

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = msg.header.frame_id if hasattr(msg, "header") else "map"

            pointcloud_msg = PointCloud2()
            pointcloud_msg.header = header
            pointcloud_msg.height = 1
            pointcloud_msg.width = point_count
            pointcloud_msg.fields = fields
            pointcloud_msg.is_bigendian = False
            pointcloud_msg.point_step = 16  # 4 * 4 bytes (float32)
            pointcloud_msg.row_step = pointcloud_msg.point_step * point_count
            pointcloud_msg.is_dense = True
            pointcloud_msg.data = data

            publisher.publish(pointcloud_msg)


def main(args=None):
    """
    Main entry point for the LanesToPointCloudNode.

    Args:
        args (list, optional): Optional arguments for rclpy.init.
    """
    rclpy.init(args=args)

    lanes_to_point_cloud_node = LanesToPointCloudNode()

    rclpy.spin(lanes_to_point_cloud_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lanes_to_point_cloud_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
