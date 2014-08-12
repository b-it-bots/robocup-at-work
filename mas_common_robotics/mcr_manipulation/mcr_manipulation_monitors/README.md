## Description
This package controls a robot manipulator, in Cartesian Space,
by publishing a twist to reduce the difference between two poses.

## Usage
1. Launch the component:    
```roslaunch mcr_manipulation_monitors cartesian_distance_monitor.launch```
2. Subscribe to the result of the component:    
```rostopic echo /cartesian_distance_monitor/event_out```
3. Publish the first pose (example):    
```rostopic pub /cartesian_distance_monitor/pose_1 geometry_msgs/PoseStamped '{header: {frame_id: "base_link"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} }}'```
4. Publish the second pose (example):    
```rostopic pub /cartesian_distance_monitor/pose_2 geometry_msgs/PoseStamped '{header: {frame_id: "base_link"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} }}'```
5. Start the component:    
```rostopic pub /cartesian_distance_monitor/event_in std_msgs/String 'e_start'```

## Usage (with mock-up)
**Note:** It assumes the robot has been brought up (to obtain the pose from a frame).

1. Launch the mock-up:    
```roslaunch mcr_manipulation_monitors pose_mock_up_gui.launch```
2. Subscribe to the result of the component:    
```rostopic echo /cartesian_distance_monitor/event_out```
3. Start the component:    
```rostopic pub /cartesian_distance_monitor/event_in std_msgs/String 'e_start'```

### To stop the component:
```rostopic pub /cartesian_distance_monitor/event_in std_msgs/String 'e_stop'```
