# Description
This package contains components to synchronize twists.

## 'twist_synchronizer' node:
This component synchronizes the velocities of a twist (represented as a
geometry_msgs/TwistStamped message), such that each component of a Cartesian error
(compensated by the twist's velocities) simultaneously reaches zero.

**Input(s):**
  * `twist`: The twist to be synchronized.
  * `pose_error`: The component-wise Cartesian difference (error).

**Output(s):**
  * `synchronized_twist`: The synchronized twist.

**Parameter(s):**
  * `angular_synchronization`: If True, it also synchronizes the angular and linear
  velocities. By default, it only synchronizes the linear velocities (bool).
  * `near_zero`: A value to prevent division by near-zero values.
  * `loop_rate`: Node cycle rate (in hz).

** Diagram **

![Twist synchronizer][twist_synchronizer]

### Usage
1. Launch the component (example):

  ```roslaunch mcr_twist_synchronizer twist_synchronizer_example.launch```

  [**Note**: You will probably need to create your own launch file and configure it
  according to your needs.]
1. Subscribe to the result(s) of the component:

  ```rostopic echo /twist_synchronizer/synchronized_twist```
1. Publish the (component-wise) pose difference (example):

  ```
  rostopic pub /twist_synchronizer/pose_error mcr_manipulation_msgs/ComponentWiseCartesianDifference '{header: {frame_id: "/arm_link_5"}, linear: {x: 0.05, y: 0.0, z: 0.02}, angular: {x: 0.0, y: 0.0, z: 0.0 }}'
  ```
1. Publish a twist velocity (example):

  ```
  rostopic pub /twist_synchronizer/twist geometry_msgs/TwistStamped '{header: {frame_id: "/arm_link_5"}, twist: {linear: {x: 0.05, y: 0.0, z: 0.02}, angular: {x: 0.0, y: 0.0, z: 0.0 }}}'
  ```

1. Toggle the component:
  1. To start the component:

    ```rostopic pub /twist_synchronizer/event_in std_msgs/String 'e_start'```
  1. To stop the component:

    ```rostopic pub /twist_synchronizer/event_in std_msgs/String 'e_stop'```

[twist_synchronizer]: https://mas.b-it-center.de/gitgate/mas-group/mas_common_robotics/tree/hydro/mcr_controllers/mcr_twist_synchronizer/ros/doc/twist_synchronizer.pdf "Twist synchronizer"
