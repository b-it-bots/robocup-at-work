# Description
This package contains components to generate a list of poses around a target pose.

## 'pose_generator' node:
This component generates a list of poses around a target pose based on a set of
parameters (SphericalSamplerParameters).

**Input(s):**
  * `target_pose`: The target pose from which to create a set of poses around that
  object.
  * `sampling_parameters`: A message specifying the parameters, and constraints,
  of the pose to be sampled around an object, if any.

**Output(s):**
  * `poses_list`: The list of poses around the target pose as defined by the
  `sampling_parameters`.

**Parameter(s):**
  * `linear_step`: Sampling step for linear variables (in meters).
  * `angular_step`: Sampling step for angular variables (in degrees).
  * `max_poses`: Maximum amount of samples to be generated (int).
  * `gripper`: Configuration matrix of the gripper to be used (as a string).
  * `loop_rate`: Node cycle rate (in hz).

** Diagram **

![Pose generator][pose_generator]

### Usage
1. Launch the component (example):

  ```roslaunch mcr_pose_generation pose_generator.launch```

  [**Note**: You will probably need to create your own launch file and configure it
  according to your needs.]
1. Subscribe to the result(s) of the component:

  ```rostopic echo /pose_generator/poses_list```
1. Define the orbit parameters for the pose generator (example):

  ```
  rostopic pub /pose_generator/sampling_parameters mcr_manipulation_msgs/SphericalSamplerParameters '{height: {minimum: 0.0, maximum: 0.0}, zenith: {minimum: 1.571, maximum: 3.2}, azimuth: {minimum: 0.0, maximum: 0.0}, yaw: {minimum: -0.524, maximum: 0.524}, radial_distance: {minimum: 0.01, maximum: 0.05}}'
  ```
1. Publish the target pose (example):

  ```
  rostopic pub /pose_generator/target_pose geometry_msgs/PoseStamped '{header: {frame_id: "base_link"}, pose: {position: {x: 0.49, y: -0.01, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} }}'
  ```
1. Toggle the component:
  1. To start the component:

    ```rostopic pub /pose_generator/event_in std_msgs/String 'e_start'```
  1. To stop the component:

    ```rostopic pub /pose_generator/event_in std_msgs/String 'e_stop'```


### Usage (with GUI)

1. Launch the component:

  ```roslaunch mcr_pose_generation pose_generator.launch```

  [**Note**: You will probably need to create your own launch file and configure it
  according to your needs.]
1. Subscribe to the result(s) of the component:

  ```rostopic echo /pose_generator/poses_list```
1. Launch the GUI:

  ```roslaunch mcr_pose_generation sampling_parameter_gui.launch```
1. Publish the target pose (example):

  ```
  rostopic pub /pose_generator/target_pose geometry_msgs/PoseStamped '{header: {frame_id: "base_link"}, pose: {position: {x: 0.49, y: -0.01, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} }}'
  ```
1. Toggle the component:
  1. To start the component:

    ```rostopic pub /pose_generator/event_in std_msgs/String 'e_start'```
  1. To stop the component:

    ```rostopic pub /pose_generator/event_in std_msgs/String 'e_stop'```

[pose_generator]: https://mas.b-it-center.de/gitgate/mas-group/mas_common_robotics/tree/hydro/mcr_common/mcr_pose_generation/ros/doc/pose_generator.pdf "Pose generator"
