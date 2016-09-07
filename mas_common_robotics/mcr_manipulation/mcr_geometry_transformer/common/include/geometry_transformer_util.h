#ifndef GEOMETRY_TRANSFROMER_UTIL_H
#define GEOMETRY_TRANSFROMER_UTIL_H

#include <tf/tf.h>
#include <geometry_msgs/WrenchStamped.h>

/**
 * Apply a transform to a wrench. It is assumed that the reference point and
 * reference frame are collapsed into a single coordinate frame. (See also
 * http://www.ros.org/wiki/tf/Reviews/2010-03-12_API_Review)
 *
 * @param transform The desired transform that should be applied.
 *
 * @param wrench_in The wrench to which the transform should be applied.
 *
 * @return The transformed wrench.
 */
geometry_msgs::WrenchStamped transform_wrench(
    const tf::StampedTransform &transform,
    const geometry_msgs::WrenchStamped &wrench_in);

#endif
