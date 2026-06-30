# Path Planning

[![Build Test](https://github.com/DHBW-Smart-Rollerz/pathplanning/actions/workflows/build-test.yaml/badge.svg)](https://github.com/DHBW-Smart-Rollerz/pathplanning/actions/workflows/build-test.yaml)

This repository contains the pathplanning package for the Smarty project. It takes the lane
detection result, fits a smooth driving path through the detected lanes and publishes the
target trajectories for the controller.

## Overview

The package contains the following nodes:

| Node | Executable | Purpose | Started by |
| --- | --- | --- | --- |
| Path Planning | `path_planning_node` | Main node. Fits the lanes and publishes the target driving paths. | launch file |
| Lanes to PointCloud | `lanes_to_pointcloud_node` | Debug only. Converts the lane detection result to colored `PointCloud2` messages for visualization in RViz. | run separately |

In addition the path planning node contains an optional **crossing detection** feature
(`crossing_interference`) that is disabled by default. See [Crossing Detection](#crossing-detection-crossing_interference).

## How It Works

The path planning node receives the lane detection result, which contains a point cloud for the
`left`, `center` and `right` lane. For each lane the following steps are performed:

1. **Filtering** – Points outside the relevant region of interest (`min_x`/`max_x` and a lateral
   limit) are removed to ignore noise on the image border.
2. **RANSAC fitting** – A 3rd degree polynomial is fitted to the remaining points using a
   `RANSACRegressor` with a `RidgeCV` base estimator. This makes the fit robust against outliers.
3. **Buffering & smoothing** – Each lane keeps a buffer of the last fits. New fits that differ too
   much from the previous result are rejected and the last valid result is reused. A weighted
   average over the buffer (newer frames weighted higher) is used to keep the path smooth while
   still reacting quickly to turns.
4. **Missing lane simulation** – If a lane is not detected, it is reconstructed by offsetting one of
   the other detected lanes by the lane width. If no lane is detected at all, the last known result
   (or a straight default path) is used.

Finally the node computes the two drivable paths as the midlines between the `left`/`center` and
`center`/`right` lanes and publishes their polynomial coefficients for the controller.

## Build

Clone the repository into your workspace and build it with colcon:

```bash
cd $ROS2_SMARTY_WORKSPACE_DIR/src
git clone https://github.com/DHBW-Smart-Rollerz/pathplanning.git
cd ..
colcon build --symlink-install --packages-select pathplanning
```

## Usage

### Path Planning Node

After building, source the workspace and launch the node:

```bash
source install/setup.bash
ros2 launch pathplanning pathplanning.launch.py
```

### Lanes to PointCloud Node (Debug)

The `lanes_to_pointcloud_node` is a pure debugging helper that publishes the detected lane points
as colored `PointCloud2` messages so the lane detection result can be inspected in RViz
(red = left, green = center, blue = right). It is **not** part of the launch file and has to be
started separately:

```bash
ros2 run pathplanning lanes_to_pointcloud_node
```

## Launch Arguments

The launch file accepts the following arguments:

- `debug`: Enable debug mode. When enabled, the node publishes additional `Marker` messages for the
  fitted lanes and paths so they can be visualized in RViz (default: `false`).
- `crossing_interference`: Enable the crossing detection feature (default: `false`,
  see [Crossing Detection](#crossing-detection-crossing_interference)).
- `params_file`: Path to the ROS parameters file (default: `ros_params.yaml` in the `config` folder).

Example:

```bash
ros2 launch pathplanning pathplanning.launch.py debug:=true crossing_interference:=true
```

## Topics

### Path Planning Node

#### Subscribers

| Topic | Type | Description |
| --- | --- | --- |
| `/lane_detection/lane` | `lane_msgs/LaneDetectionResult` | Detected lane point clouds (left, center, right). |
| `/state_machine/path_planning/direction` | `std_msgs/String` | Desired crossing direction. Only subscribed when `crossing_interference` is enabled. |

#### Publishers

| Topic | Type | Description |
| --- | --- | --- |
| `/path_planning/target/left` | `std_msgs/Float32MultiArray` | Polynomial coefficients of the left driving path (midline between left and center lane). |
| `/path_planning/target/right` | `std_msgs/Float32MultiArray` | Polynomial coefficients of the right driving path (midline between center and right lane). |
| `/path_planning/debug/left` | `visualization_msgs/Marker` | Fitted left lane (debug only). |
| `/path_planning/debug/center` | `visualization_msgs/Marker` | Fitted center lane (debug only). |
| `/path_planning/debug/right` | `visualization_msgs/Marker` | Fitted right lane (debug only). |
| `/path_planning/debug/left_path` | `visualization_msgs/Marker` | Left driving path (debug only). |
| `/path_planning/debug/right_path` | `visualization_msgs/Marker` | Right driving path (debug only). |

### Lanes to PointCloud Node

#### Subscribers

| Topic | Type | Description |
| --- | --- | --- |
| `/lane_detection/lane` | `lane_msgs/LaneDetectionResult` | Detected lane point clouds (left, center, right). |

#### Publishers

| Topic | Type | Description |
| --- | --- | --- |
| `/pathplanning/lanes/left` | `sensor_msgs/PointCloud2` | Left lane points (red). |
| `/pathplanning/lanes/center` | `sensor_msgs/PointCloud2` | Center lane points (green). |
| `/pathplanning/lanes/right` | `sensor_msgs/PointCloud2` | Right lane points (blue). |

## ROS Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `debug` | `false` | Enable debug output and RViz `Marker` publishing. |
| `crossing_interference` | `false` | Enable the crossing detection feature. |

## Crossing Detection (`crossing_interference`)

The crossing detection feature is **disabled by default** and was **not used during the Caudri challenge**, because our implementation did not work reliably enough in practice. The code
is intentionally kept in the repository, as we still consider the underlying idea promising: using
the result direction derived from **direction signs** is a good way to select the correct path
through a crossing.

When enabled (`crossing_interference:=true`):

- The node subscribes to `/state_machine/path_planning/direction`, where the state machine publishes
  the desired direction (`left`, `right` or `straight`) based on the detected direction signs.
- During a crossing, lane fits that do not point towards the desired direction are discarded, so the
  car follows the path leading into the correct branch.
- If no lane points in the correct direction, a set of predefined crossing coefficients is used
  (mirrored for left/right turns).
- A short timer keeps the crossing state active for a moment after the direction switches back to
  `straight`, to bridge the gap while leaving the crossing.

## License

This repository is licensed under the MIT license. See [LICENSE](LICENSE) for details.
