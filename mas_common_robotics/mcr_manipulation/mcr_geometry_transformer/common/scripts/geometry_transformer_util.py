import geometry_msgs.msg
import numpy


def transform_wrench(transform, wrench):
    '''
    Apply a transform to a wrench. It is assumed that the reference point and
    reference frame are collapsed into a single coordinate frame. (See also
    http://www.ros.org/wiki/tf/Reviews/2010-03-12_API_Review)
    
    :param transform: The desired transform that should be applied.
    :type transform: numpy.matrix[4][4]
    
    :param wrench_in: The wrench to which the transform should be applied.
    :type wrench_in: geometry_msgs.msg.WrenchStamped]
    
    :return: The transformed wrench.
    :rtype: geometry_msgs.msg.WrenchStamped
    '''
    
    wrench_out = geometry_msgs.msg.WrenchStamped()
    
    force_in = numpy.array([wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z])
    torque_in = numpy.array([wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z])
    
    M = transform[0:3, 0:3]
    p = transform[0:3, 3]
    
    force_out = numpy.dot(M, force_in)
    torque_out = numpy.dot(M, torque_in) + numpy.cross(p, force_out)
    
    wrench_out.wrench.force.x = force_out[0]
    wrench_out.wrench.force.y = force_out[1]
    wrench_out.wrench.force.z = force_out[2]
    wrench_out.wrench.torque.x = torque_out[0]
    wrench_out.wrench.torque.y = torque_out[1]
    wrench_out.wrench.torque.z = torque_out[2]
    
    return wrench_out