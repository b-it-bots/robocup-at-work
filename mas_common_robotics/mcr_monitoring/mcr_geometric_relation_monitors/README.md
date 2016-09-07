## Description
This package monitors the geometric relations

## Use Case
This package can be used to monitor Component Wise Pose Calculator node which finds the component wise pose error between two geometry_msgs/PoseStamped messages.

## Usage
1. Launch the component:
```roslaunch mcr_geometric_relation_monitors component_wise_pose_error_monitor.launch```
2. Subscribe to the feedback of the component:
```rostopic echo /mcr_monitoring/component_wise_pose_error_monitor/feedback```
3. Subscribe to the event out of the component:
```rostopic echo /mcr_monitoring/component_wise_pose_error_monitor/event_out```
4. Publish the component wise pose error (mcr_manipulation_msgs::ComponentWiseCartesianDifference) in ```/component_wise_pose_error_calculator/pose_error```

### To start the component:
```rostopic pub /mcr_monitoring/component_wise_pose_error_monitor/event_in std_msgs/String 'e_start'```

### To stop the component:
```rostopic pub /mcr_monitoring/component_wise_pose_error_monitor/event_in std_msgs/String  'e_stop'```