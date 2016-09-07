# Description
This package contains components to generate trajectories based
on different methods (e.g. using Dynamic Movement Primitives).

## 'linear_interpolator' node:
This component generates a path (in task space) to reach a goal pose from a
start pose based on linear interpolation. It is assumed that the start and goal
poses have the same reference frame.

**Input(s):**
  * `start_pose`: The pose where the manipulator's end effector will start the
  trajectory.
  * `goal_pose`: The pose where the manipulator's end effector will end the trajectory.

**Output(s):**
  * `path`: The path connecting the start and goal poses on a straight line.

**Relevant parameter(s):**
  * `max_poses`: Number of poses to generate between the start and goal pose
  (including those two), e.g. specifying 5 'max_poses' will output the start pose
  as the first pose, then three poses equally separated, and finally the goal pose
  as the fifth pose.

** Diagram **
![Linear interpolator][linear_interpolator]

### Usage
1. Launch the component (example):

  ```roslaunch mcr_trajectory_generation linear_interpolator_example.launch```

  [**Note**: You will probably need to create your own launch file and configure it according to your needs.]
1. Subscribe to the result(s) of the component:

  ```rostopic echo /mcr_manipulation/linear_interpolator/path```

  ```rostopic echo /mcr_manipulation/linear_interpolator/event_out```
1. Publish the start pose (example):

  ```
  rostopic pub /mcr_manipulation/linear_interpolator/start_pose geometry_msgs/PoseStamped '{header: {frame_id: "base_link"}, pose: {position: {x: 0.3824, y: -0.0048, z: 0.3492}, orientation: {x: -0.0007, y: 0.945, z: -0.0082, w: 0.327} }}'
  ```
1. Publish the goal pose (example):

  ```
  rostopic pub /mcr_manipulation/linear_interpolator/goal_pose geometry_msgs/PoseStamped '{header: {frame_id: "base_link"}, pose: {position: {x: 0.5858, y: 0.0049, z: 0.0871}, orientation: {x: -0.0066, y: 0.7067, z: 0.0063, w: 0.7075} }}'
  ```
1. Toggle the component:
  1. To start the component:

      ```rostopic pub /mcr_manipulation/linear_interpolator/event_in std_msgs/String "e_start"```
  1. To stop the component:

    ```rostopic pub /mcr_manipulation/linear_interpolator/event_in std_msgs/String "e_stop"```

## 'dmp_based_task_space' node:
This component generates a trajectory (in task space) to reach a goal pose from a
start pose based on an example trajectory (primitive_motion). It uses
Dynamic Movement Primitives (DMPs) allowing arbitrary start/target configurations [1].

[1] http://wiki.ros.org/dmp

**Input(s):**
  * `start_pose`: The pose where the manipulator's end effector will start the
  trajectory.
  * `goal_pose`: The pose where the manipulator's end effector will end the trajectory.

**Output(s):**
  * `trajectory`: The trajectory connecting the start and goal poses, based on the
  example trajectory.

**Relevant parameter(s):**
  * `primitive_motion`: Example trajectory showing a motion (e.g. grasp, place).
  This can be saved in a config file (see example config file).
  * `reference_frame`: Frame of reference for the generated trajectory.
  * `duration`: Desired duration of the trajectory (scaled to the duration of the
  demonstrated trajectory, e.g. a value of 2 will generate a plan twice as long as
  the demonstrated one).

** Diagram **
![DMP based task space][dmp_based_task_space]

### Usage
1. Verify that the DMP server is running. If not, launch it:

  ```roslaunch dmp dmp.launch```
1. Launch the component (example):

  ```roslaunch mcr_trajectory_generation dmp_based_task_space_example.launch```

  [**Note**: You will probably need to create your own launch file and configure it according to your needs.]
1. Subscribe to the result(s) of the component:

  ```rostopic echo /mcr_manipulation/dmp_based_task_space/trajectory```
1. Publish the start pose (example):

  ```
  rostopic pub /mcr_manipulation/dmp_based_task_space/start_pose geometry_msgs/PoseStamped '{header: {frame_id: "base_link"}, pose: {position: {x: 0.3824, y: -0.0048, z: 0.3492}, orientation: {x: -0.0007, y: 0.945, z: -0.0082, w: 0.327} }}'
  ```
1. Publish the goal pose (example):

  ```
  rostopic pub /mcr_manipulation/dmp_based_task_space/goal_pose geometry_msgs/PoseStamped '{header: {frame_id: "base_link"}, pose: {position: {x: 0.5858, y: 0.0049, z: 0.0871}, orientation: {x: -0.0066, y: 0.7067, z: 0.0063, w: 0.7075} }}'
  ```
1. Toggle the component:
  1. To start the component:

      ```rostopic pub /mcr_manipulation/dmp_based_task_space/event_in std_msgs/String "e_start"```
  1. To stop the component:

    ```rostopic pub /mcr_manipulation/dmp_based_task_space/event_in std_msgs/String "e_stop"```
1. Additionally you can use Dynamic Reconfigure [2] to modify the parameters
`primitive_motion` and `duration` online. For the changes to take place you will first
need to stop the component and then start it again (see the toggle the component
instructions).

  [2] http://wiki.ros.org/dynamic_reconfigure

## 'ik_trajectory_solver' node:
This component computes the inverse kinematics (IK) solution for each pose in a list
of poses.

**Input(s):**
  * `poses`: The poses for which a solution will be calculated.

**Output(s):**
  * `trajectory`: The trajectory, in joint space, specifying only the positions for the
  trajectory.

**Relevant parameter(s):**
  * `max_poses`: Maximum amount of poses to compute a trajectory for.

** Diagram **
![IK trajectory solver][ik_trajectory_solver]

## 'trajectory_generator' node:
This component computes the joint velocities and accelerations given the joint positions
of a trajectory.

**Input(s):**
  * `trajectory_in`: The trajectory, in joint space, specifying only the positions for the
  trajectory.

**Output(s):**
  * `trajectory_out`: The trajectory, in joint space, specifying the positions, velocities
   and accelerations; as well as the duration for each point.

**Relevant parameter(s):**
  * `scaling_factor`: Time scaling factor to control the duration of moving from point 1 to
  point 2 (in seconds).
  * `max_velocity`: Maximum joint velocity allowed.
  * `max_acceleration`: Maximum joint acceleration allowed.

** Diagram **
![Trajectory generator][trajectory_generator]

## 'trajectory_executor' node:
This component executes a specified trajectory.

**Input(s):**
  * `trajectory_in`: The trajectory, in joint space, specifying the positions, velocities
   and accelerations; as well as the duration for each point.

**Output(s):**
  * `trajectory_out`: The specified trajectory as a trajectory goal.

**Relevant parameter(s):**
  * `trajectory_controller`: Trajectory controller to be used to execute the trajectory

** Diagram **
![Trajectory executor][trajectory_executor]


## 'linear_interpolator_demo' node:
A demo to generate a trajectory using linear interpolation and subsequently moving
the arm to follow such trajectory.

This component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.

**Input(s):**
  * `start_pose`: The pose where the manipulator's end effector will start the
  trajectory.
  * `goal_pose`: The pose where the manipulator's end effector will end the trajectory.

**Output(s):**
  * `trajectory`: The trajectory, in joint space, connecting the start and goal poses as
  a goal.

** Diagram **
![Linear interpolator demo][linear_interpolator_demo]

### Usage
1. Launch the component (example):

  ```roslaunch mcr_trajectory_generation linear_interpolator_demo.launch ```

  [**Note**: You will probably need to create your own launch file and configure it
  according to your needs.]
1. Subscribe to the result(s) of the component:

  ```rostopic echo /mcr_manipulation/linear_interpolator_demo/event_out```
1. Publish the start pose (example):

  ```
  rostopic pub /mcr_manipulation/linear_interpolator_demo/start_pose geometry_msgs/PoseStamped "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.3824, y: -0.0048, z: 0.3492}, orientation: {x: -0.0007, y: 0.945, z: -0.0082, w: 0.327} }}"
  ```
1. Publish the goal pose (example):

  ```
  rostopic pub /mcr_manipulation/linear_interpolator_demo/goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.42, y: -0.0048, z: 0.30}, orientation: {x: -0.0007, y: 0.945, z: -0.0082, w: 0.327} }}"
  ```

1. Toggle the component:
  1. To start the component:

      ```rostopic pub /mcr_manipulation/linear_interpolator_demo/event_in std_msgs/String "e_start"```
  1. To stop the component:

      ```rostopic pub /mcr_manipulation/linear_interpolator_demo/event_in std_msgs/String "e_stop"```
1. If the result of the component was a `e_success`, then the arm can be moved with
the following command:

  ```rostopic pub /mcr_manipulation/linear_interpolator_demo_trajectory_executor/event_in std_msgs/String "e_start"```

## 'dmp_trajectory_demo' node:
A demo to generate a trajectory using DMPs and subsequently moving the arm to follow
such trajectory. This component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.

**Input(s):**
  * `start_pose`: The pose where the manipulator's end effector will start the
  trajectory.
  * `goal_pose`: The pose where the manipulator's end effector will end the trajectory.

**Output(s):**
  * `trajectory`: The trajectory, in joint space, connecting the start and goal poses as
  a goal.

** Diagram **
![DMP trajectory demo][dmp_trajectory_demo]

### Usage
1. Launch the component (example):

  ```roslaunch mcr_trajectory_generation dmp_trajectory_demo.launch ```

  [**Note**: You will probably need to create your own launch file and configure it
  according to your needs.]
1. Subscribe to the result(s) of the component:

  ```rostopic echo /mcr_manipulation/dmp_trajectory_demo/event_out```
1. Publish the start pose (example):

  ```
  rostopic pub /mcr_manipulation/dmp_trajectory_demo/start_pose geometry_msgs/PoseStamped "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.3824, y: -0.0048, z: 0.3492}, orientation: {x: -0.0007, y: 0.945, z: -0.0082, w: 0.327} }}"
  ```
1. Publish the goal pose (example):

  ```
  rostopic pub /mcr_manipulation/dmp_trajectory_demo/goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.42, y: -0.0048, z: 0.30}, orientation: {x: -0.0007, y: 0.945, z: -0.0082, w: 0.327} }}"
  ```

1. Toggle the component:
  1. To start the component:

      ```rostopic pub /mcr_manipulation/dmp_trajectory_demo/event_in std_msgs/String "e_start"```
  1. To stop the component:

      ```rostopic pub /mcr_manipulation/dmp_trajectory_demo/event_in std_msgs/String "e_stop"```
1. If the result of the component was a `e_success`, then the arm can be moved with
the following command:

  ```rostopic pub /mcr_manipulation/dmp_demo_trajectory_executor/event_in std_msgs/String "e_start"```

[linear_interpolator]: https://mas.b-it-center.de/gitgate/mas-group/mas_common_robotics/tree/hydro/mcr_manipulation/mcr_trajectory_generation/ros/doc/linear_interpolator.png "Linear interpolator"
[dmp_based_task_space]: https://mas.b-it-center.de/gitgate/mas-group/mas_common_robotics/tree/hydro/mcr_manipulation/mcr_trajectory_generation/ros/doc/dmp_based_task_space.png "DMP based task space"
[ik_trajectory_solver]: https://mas.b-it-center.de/gitgate/mas-group/mas_common_robotics/tree/hydro/mcr_manipulation/mcr_trajectory_generation/ros/doc/ik_trajectory_solver.png "IK trajectory solver"
[trajectory_generator]: https://mas.b-it-center.de/gitgate/mas-group/mas_common_robotics/tree/hydro/mcr_manipulation/mcr_trajectory_generation/ros/doc/trajectory_generator.png "Trajectory generator"
[trajectory_executor]: https://mas.b-it-center.de/gitgate/mas-group/mas_common_robotics/tree/hydro/mcr_manipulation/mcr_trajectory_generation/ros/doc/trajectory_executor.png "Trajectory executor"
[linear_interpolator_demo]: https://mas.b-it-center.de/gitgate/mas-group/mas_common_robotics/tree/hydro/mcr_manipulation/mcr_trajectory_generation/ros/doc/linear_interpolator_demo.png "Linear interpolator demo"
[dmp_trajectory_demo]: https://mas.b-it-center.de/gitgate/mas-group/mas_common_robotics/tree/hydro/mcr_manipulation/mcr_trajectory_generation/ros/doc/dmp_trajectory_demo.png "DMP trajectory demo"
