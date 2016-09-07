# Name of the cavity
string name

# Object that can be placed in this cavity
string object_name

# Possible orientations of object when being placed in the cavity
uint8 PLACEMENT_HORIZONTAL=0
uint8 PLACEMENT_VERTICAL=1
uint8 PLACEMENT_HORIZONTAL_OR_VERTICAL=2

# Orientation of object that can be placed in the cavity
uint8 placement_orientation

# Pose of the cavity
geometry_msgs/PoseStamped pose

# Dimensions of the cavity
geometry_msgs/Vector3Stamped dimensions

# Error when matching contour of cavity with a given template
MatchingErrorStamped template_matching_error

# Pointcloud of the cavity
sensor_msgs/PointCloud2 pointcloud
