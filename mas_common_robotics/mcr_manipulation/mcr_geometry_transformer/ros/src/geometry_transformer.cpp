#include <mcr_geometry_transformer/geometry_transformer.h>
#include <geometry_transformer_util.h>


GeometryTransformer::GeometryTransformer()
{
}


GeometryTransformer::~GeometryTransformer()
{
}


geometry_msgs::WrenchStamped GeometryTransformer::transformWrench(
		const std::string &target_frame,
		const geometry_msgs::WrenchStamped &wrench_in)
{
	// wait for the transform
	tf::StampedTransform transform;
	_listener.lookupTransform(target_frame, wrench_in.header.frame_id,
			wrench_in.header.stamp, transform);

	return transform_wrench(transform, wrench_in);
}
