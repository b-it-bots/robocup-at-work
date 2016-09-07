## Description
This package transforms the image pixel pose to cartesian space with respect to the required frame

## Usage
1. Launch the component:
```roslaunch mcr_image_cartesian_mapper image_cartesian_mapper.launch```
2. Subscribe to the result of the component:
```rostopic echo /mcr_perception/image_cartesian_mapper/cartesian_pose```
3. Subscribe to the event out of the component:
```rostopic echo /mcr_perception/image_cartesian_mapper/event_out```
4. Publish the pixel pose(geometry_msgs::Pose2D) in /mcr_perception/blob_tracker/blob_pose topic
5. Publish the camera_info(sensor_msgs::CameraInfo) in /arm_cam3d/rgb/camera_info topic 
6. Make sure that the camera calibration yaml file loaded in the launch file is pointing to the correct file
7. Make sure that the image filter params yaml file has the same values as that of the image filter dynamic reconfigure server defaults 

### To start the component:
```rostopic pub /mcr_perception/image_cartesian_mapper/event_in std_msgs/String 'e_start'```

### To stop the component:
```rostopic pub /mcr_perception/image_cartesian_mapper/event_in std_msgs/String  'e_stop'```

### Camera calibration:
Use camera_calibration ros node. Tutorials are provided for calibrating monocular cameras in http://wiki.ros.org/camera_calibration