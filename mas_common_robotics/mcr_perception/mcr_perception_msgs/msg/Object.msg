# The pose where the object can be found
geometry_msgs/PoseStamped pose

# The dimensions of the object
geometry_msgs/Vector3Stamped dimensions

# Name of the object determined e.g. by the recognition
string name

# Categorization of this object
string category

# The object's id in a database
int32 database_id

# Probability 
float32 probability

# Extracted RGB image of the image
sensor_msgs/Image rgb_image

# Segmented cluster that represents the object
sensor_msgs/PointCloud2 pointcloud

# The bounding box of the object
mcr_perception_msgs/BoundingBox bounding_box

# The surface that the object was found on
mcr_perception_msgs/Plane related_surface
