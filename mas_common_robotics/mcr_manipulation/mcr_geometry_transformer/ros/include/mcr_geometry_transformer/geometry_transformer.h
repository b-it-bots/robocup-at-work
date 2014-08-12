#ifndef GEOMETRY_TRANSFORMER_H
#define GEOMETRY_TRANSFORMER_H

// ROS includes
#include <tf/transform_listener.h>
#include <geometry_msgs/WrenchStamped.h>
#include <kdl/frames.hpp>


class GeometryTransformer
{
	public:
		/**
		 * Ctor.
		 */
		GeometryTransformer();

		/**
		 * Dtor.
		 */
		virtual ~GeometryTransformer();

		/**
		 * Transform a provided wrench to the target frame.
		 */
		geometry_msgs::WrenchStamped transformWrench(
				const std::string &target_frame,
				const geometry_msgs::WrenchStamped &wrench_in);


	private:
		/**
		 * Provide access to the ROS transforms.
		 */
		tf::TransformListener _listener;
};

#endif
