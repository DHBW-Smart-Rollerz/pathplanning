# Path Planning

[![Build Test](https://github.com/DHBW-Smart-Rollerz/pathplanning/actions/workflows/build-test.yaml/badge.svg)](https://github.com/DHBW-Smart-Rollerz/pathplanning/actions/workflows/build-test.yaml)

This repository contains the pathplanning package and messages for the Smarty project.

## Usage

To use this package, clone the repository into your workspace and build it with colcon.

```bash
cd $ROS2_SMARTY_WORKSPACE_DIR/src
git clone https://github.com/DHBW-Smart-Rollerz/pathplanning.git
cd ..
colcon build --symlink-install --packages-select pathplanning
```

After building the package, you can source the workspace and run the nodes.

```bash
source install/setup.bash
ros2 launch pathplanning pathplanning.launch.py
```

## Launch Arguments

The launch file accepts the following arguments:

- `debug`: Enable debug mode (default: false)
- `params_file`: Path to the ROS parameters file (default: ros_params.yaml in the config folder)

## ROS Parameters

The package uses the following ROS parameters:

### Subscribers

- **lane_points_subscriber**: Topic to subscribe to for receiving lane detection results.
  - Default: `'/lane_detection/result'`

- **image_subscriber**: Topic to subscribe to for receiving bird's eye view images.
  - Default: `'/camera/birds_eye'`

- **state_machine_subscriber**: Topic to subscribe to for receiving state machine debug information.
  - Default: `'/state_machine/debug'`

- **state_machine_lane_subscriber**: Topic to subscribe to for receiving right lane information from the state machine.
  - Default: `'/rightLane'`

- **remote_state_subscriber**: Topic to subscribe to for receiving remote state information.
  - Default: `'/remoteState'`

- **targetSteeringAngle_pub**: Topic to publish the target steering angle.
  - Default: `'/targetSteeringAngle'`

### Publishers

- **image_debug_publisher**: Topic to publish debug images for path planning.
  - Default: `'/pathplanning/debug_image'`

- **path_planning_right_publisher**: Topic to publish the right lane coefficients for path planning.
  - Default: `'/pathplanning/right_lane_coefficients'`

- **path_planning_left_publisher**: Topic to publish the left lane coefficients for path planning.
  - Default: `'/pathplanning/left_lane_coefficients'`

- **ref_point_publisher**: Topic to publish the reference pose for the controller.
  - Default: `'/controller/ref_pose'`

### Additional Parameters

- **trj_look_forward**: Number of points to look forward in the trajectory. This parameter should be adapted to speed and runtime.
  - Default: `100`

- **active**: Boolean flag to indicate whether the node is active.
  - Default: `True`

## License

This repository is licensed under the MIT license. See [LICENSE](LICENSE) for details.
