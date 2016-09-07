# Description
This package that provides event-based wrappers that serve as interfaces to move
a robot manipulator arm in a specific space (e.g. cartesian space, joint space).

## 'planned_motion' node:
This component moves a robotic manipulator, in a planned manner, to a specified
joint configuration using MoveIt!.

**Input(s):**
  * `target_configuration`: The joint configuration to which the manipulator will
  be moved.

**Parameter(s):**
  * `move_group`: MoveIt! interface.
  * `arm`: Name of the group to move.
  * `loop_rate`: Node cycle rate (in hz).

**Diagram**

![Planned motion][planned_motion]

## 'planned_motion_with_feedback' node:
This component uses the `planned_motion` component and it provides feedback
whether the desired configuration is reached.

It uses the following nodes:
  * (mcr_arm_motions) `planned_motion`.
  * (mcr_topic_tools) `brics_joints_to_joint_states`.
  * (mcr_manipulation_monitors) `joint_position_monitor`.

The component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.

**Input(s):**
  * `target_configuration`: The joint configuration to which the manipulator will
  be moved.

**Parameter(s):**
  * `joint_position_monitor`
    * `target_joint_names`: Names of the joints to be monitored.
    * `epsilon`: Tolerance, as joint position value, for each joint.

**Diagram**

![Planned motion with feedback][planned_motion_with_feedback]

[planned_motion]: https://mas.b-it-center.de/gitgate/mas-group/mas_common_robotics/tree/hydro/mir_manipulation/mcr_manipulation/mcr_arm_motions/ros/doc/planned_motion.pdf "Planned motion"
[planned_motion_with_feedback]: https://mas.b-it-center.de/gitgate/mas-group/mas_common_robotics/tree/hydro/mir_manipulation/mcr_manipulation/mcr_arm_motions/ros/doc/planned_motion_with_feedback.pdf "Planned motion with feedback"
