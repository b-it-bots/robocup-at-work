# This message is a wrapper for the PCL datatype PlanarPolygon.
# http://docs.pointclouds.org/trunk/classpcl_1_1_planar_polygon.html

# Header
Header header
# Plane coefficients
float32[4] coefficients
# List of points in the contour
geometry_msgs/Point32[] contour
