rostopic pub -r 10 /mcr_manipulation/mcr_arm_cartesian_control/cartesian_velocity_command geometry_msgs/TwistStamped '{header: {frame_id: /base_link}, twist: {linear: {z: -0.05}}}'
