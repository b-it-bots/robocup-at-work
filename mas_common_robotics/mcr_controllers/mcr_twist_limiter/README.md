## Description
This package sets the individual components of a twist
(represented as a geometry_msgs/TwistStamped message) to
a specified maximum, if they exceed their respective limit.

## Usage
1. Launch the component:    
```roslaunch mcr_twist_limiter twist_limiter.launch```
2. Subscribe to the result of the component:    
```rostopic echo /twist_limiter/limited_twist```
3. Publish a twist velocity (example):    
```rostopic pub /twist_limiter/twist geometry_msgs/TwistStamped '{header: {frame_id: "/arm_link_5"}, twist: {linear: {x: 0.05, y: 0.0, z: 0.02}, angular: {x: 0.0, y: 0.0, z: 0.0 }}}'```
4. Start the component:    
```rostopic pub /twist_limiter/event_in std_msgs/String 'e_start'```

## Usage (with simulator GUI)
1. Launch the simulator GUI:    
```roslaunch mcr_twist_limiter simulated_twist_gui.launch```
2. Launch the component:    
```roslaunch mcr_twist_limiter twist_limiter.launch```
3. Subscribe to the result of the component:    
```rostopic echo /twist_limiter/limited_twist```
4. Start the component:    
```rostopic pub /twist_limiter/event_in std_msgs/String 'e_start'```

### To stop the component:
```rostopic pub /twist_limiter/event_in std_msgs/String  'e_stop'```
