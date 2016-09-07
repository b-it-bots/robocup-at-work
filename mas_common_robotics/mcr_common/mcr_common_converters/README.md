# Description
A package to convert different messages (e.g. a TF to a PoseStamped message).

## 'transform_to_pose_converter'
This module contains a component that converts the
ROS-TF transform between two frames into a ROS pose message.

### Usage
**Note:** It requires a robot to be running (i.e. TF must be available).

1. Launch the component:
```roslaunch mcr_common_converters transform_to_pose_converter.launch```
**Note**: You will probably need to create your own launch file and configure
it according to your needs.launch file for your needs.
2. Subscribe to the result of the component:
```rostopic echo /transform_to_pose_converter/converted_pose```
5. Start the component:
```rostopic pub /transform_to_pose_converter/event_in std_msgs/String "e_start"```
6. To stop the component:
```rostopic pub /transform_to_pose_converter/event_in std_msgs/String "e_stop"```