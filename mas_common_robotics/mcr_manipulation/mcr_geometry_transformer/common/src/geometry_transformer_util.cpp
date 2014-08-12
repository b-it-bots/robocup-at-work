#include <geometry_transformer_util.h>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames.hpp>


geometry_msgs::WrenchStamped transform_wrench(
		const tf::StampedTransform &transform,
		const geometry_msgs::WrenchStamped &wrench_in)
{
	geometry_msgs::WrenchStamped wrench_out;

	KDL::Frame kdl_frame;
	KDL::Wrench kdl_wrench;

	// Create the KDL data types
	tf::transformTFToKDL(transform, kdl_frame);
	tf::wrenchMsgToKDL(wrench_in.wrench, kdl_wrench);

	// Perform the transform
	KDL::Wrench transforemd_wrench = kdl_frame * kdl_wrench;

	// Create the ROS data types
	wrench_out.header.stamp = wrench_in.header.stamp;
	wrench_out.header.frame_id = transform.frame_id_;
	tf::wrenchKDLToMsg(transforemd_wrench, wrench_out.wrench);

	return wrench_out;
}
