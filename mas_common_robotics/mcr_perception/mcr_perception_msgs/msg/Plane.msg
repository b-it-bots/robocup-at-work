# The pose gives you the transform that take you to the coordinate system
# of the plane, with the origin somewhere in the plane and the 
# z axis normal to the plane
geometry_msgs/PoseStamped pose

# Point + normal vector of the plane
geometry_msgs/PointStamped plane_point
geometry_msgs/Vector3Stamped normal

# These values give you the observed extents of the plane, along x and y,
# in the plane's own coordinate system (above)
# there is no guarantee that the origin of the plane coordinate system is
# inside the boundary defined by these values. 
float32 min_x
float32 max_x
float32 min_y
float32 max_y

#geometry_msgs/Point32 top_left
#geometry_msgs/Point32 top_right

#geometry_msgs/Point32 bottom_left
#geometry_msgs/Point32 bottom_right

# There is no guarantee that the plane does NOT extend further than these 
# values; this is just as far as we've observed it.