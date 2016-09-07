# Description
This package generates a list of poses of a manipulator's
end-effector around an object's pose.

### Usage
1. Launch the 'pose_generator':    
```roslaunch mcr_pose_generator pose_generator.launch```
2. Subscribe to the result of the 'pose_generator':    
```rostopic echo /pose_generator/poses_list```
3. Define the orbit parameters for the pose generator (YouBot example):    
```rostopic pub /pose_generator/sampling_parameters mcr_manipulation_msgs/SphericalSamplerParameters '{height: {minimum: 0.0, maximum: 0.0}, zenith: {minimum: 1.571, maximum: 3.2}, azimuth: {minimum: 0.0, maximum: 0.0}, yaw: {minimum: -0.524, maximum: 0.524}, radial_distance: {minimum: 0.01, maximum: 0.05}}'```
4. Publish the target pose (example):    
```rostopic pub /pose_generator/target_pose geometry_msgs/PoseStamped '{header: {frame_id: 'base_link'}, pose: {position: {x: 0.49, y: -0.01, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} }}'```
5. Start the 'pose_generator':    
```rostopic pub /pose_generator/event_in std_msgs/String 'e_start'```
6. To stop the 'pose_generator':
```rostopic pub /pose_generator/event_in std_msgs/String 'e_stop'```


### Usage (with GUI)

1. Launch the component:
```roslaunch mcr_pose_generator pose_generator.launch```
2. Subscribe to the result of the 'pose_generator':
```rostopic echo /pose_generator/poses_list```
3. Launch the GUI:
```roslaunch mcr_pose_generator sampling_parameter_gui.launch```
4. Publish the target pose (example):
```rostopic pub /pose_generator/target_pose geometry_msgs/PoseStamped '{header: {frame_id: 'base_link'}, pose: {position: {x: 0.49, y: -0.01, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} }}'```
5. Start the 'pose_generator':
```rostopic pub /pose_generator/event_in std_msgs/String 'e_start'```
6. To stop the 'pose_generator':
```rostopic pub /pose_generator/event_in std_msgs/String 'e_stop'```
