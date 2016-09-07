#!/usr/bin/env python

import math
import numpy
import unittest
import geometry_msgs.msg
import geometry_transformer_util.geometry_transformer_util


class TestWrenchTransformer(unittest.TestCase):

    def test_identity_transform(self):
        wrench_in = geometry_msgs.msg.WrenchStamped()
        transform = numpy.identity(4)
        
        wrench_in.wrench.force.x = 0.0
        wrench_in.wrench.force.y = 0.0
        wrench_in.wrench.force.z = 0.0
        wrench_in.wrench.torque.x = 0.0
        wrench_in.wrench.torque.y = 0.0
        wrench_in.wrench.torque.z = 0.0
        w = geometry_transformer_util.geometry_transformer_util.transform_wrench(transform, wrench_in)
        self.assertEquals(0.0, w.wrench.force.x)
        self.assertEquals(0.0, w.wrench.force.y)
        self.assertEquals(0.0, w.wrench.force.z)
        self.assertEquals(0.0, w.wrench.torque.x)
        self.assertEquals(0.0, w.wrench.torque.y)
        self.assertEquals(0.0, w.wrench.torque.z)
        
        wrench_in.wrench.force.x = 1.0
        wrench_in.wrench.force.y = 2.0
        wrench_in.wrench.force.z = 3.0
        wrench_in.wrench.torque.x = 4.0
        wrench_in.wrench.torque.y = 5.0
        wrench_in.wrench.torque.z = 6.0
        w = geometry_transformer_util.geometry_transformer_util.transform_wrench(transform, wrench_in)
        self.assertEquals(1.0, w.wrench.force.x)
        self.assertEquals(2.0, w.wrench.force.y)
        self.assertEquals(3.0, w.wrench.force.z)
        self.assertEquals(4.0, w.wrench.torque.x)
        self.assertEquals(5.0, w.wrench.torque.y)
        self.assertEquals(6.0, w.wrench.torque.z)
        
        
    def test_translation_transform(self):
        wrench_in = geometry_msgs.msg.WrenchStamped()
        transform = numpy.identity(4)
        
        transform[0, 3] = 1.0
        transform[1, 3] = 2.0
        transform[2, 3] = 3.0
        
        wrench_in.wrench.force.x = 0.0
        wrench_in.wrench.force.y = 0.0
        wrench_in.wrench.force.z = 0.0
        wrench_in.wrench.torque.x = 0.0
        wrench_in.wrench.torque.y = 0.0
        wrench_in.wrench.torque.z = 0.0
        w = geometry_transformer_util.geometry_transformer_util.transform_wrench(transform, wrench_in)
        self.assertEquals(0.0, w.wrench.force.x)
        self.assertEquals(0.0, w.wrench.force.y)
        self.assertEquals(0.0, w.wrench.force.z)
        self.assertEquals(0.0, w.wrench.torque.x)
        self.assertEquals(0.0, w.wrench.torque.y)
        self.assertEquals(0.0, w.wrench.torque.z)
        
        transform[0, 3] = 1.0
        transform[1, 3] = 0.0
        transform[2, 3] = 0.0
        
        wrench_in.wrench.force.x = 0.0
        wrench_in.wrench.force.y = 1.0
        wrench_in.wrench.force.z = 0.0
        wrench_in.wrench.torque.x = 0.0
        wrench_in.wrench.torque.y = 0.0
        wrench_in.wrench.torque.z = 0.0
        w = geometry_transformer_util.geometry_transformer_util.transform_wrench(transform, wrench_in)
        self.assertEquals(0.0, w.wrench.force.x)
        self.assertEquals(1.0, w.wrench.force.y)
        self.assertEquals(0.0, w.wrench.force.z)
        self.assertEquals(0.0, w.wrench.torque.x)
        self.assertEquals(0.0, w.wrench.torque.y)
        self.assertEquals(1.0, w.wrench.torque.z)
        
        
    def test_orientation_roll_transform(self):
        wrench_in = geometry_msgs.msg.WrenchStamped()
        angle = math.pi / 2.0
        transform = numpy.array([
            [1,               0,                0, 0],
            [0, math.cos(angle), -math.sin(angle), 0],
            [0, math.sin(angle),  math.cos(angle), 0],
            [0,               0,                0, 1]])
        
        wrench_in.wrench.force.x = 1.0
        wrench_in.wrench.force.y = 0.0
        wrench_in.wrench.force.z = 0.0
        wrench_in.wrench.torque.x = 1.0
        wrench_in.wrench.torque.y = 0.0
        wrench_in.wrench.torque.z = 0.0
        w = geometry_transformer_util.geometry_transformer_util.transform_wrench(transform, wrench_in)
        self.assertEquals(1.0, w.wrench.force.x)
        self.assertEquals(0.0, w.wrench.force.y)
        self.assertEquals(0.0, w.wrench.force.z)
        self.assertEquals(1.0, w.wrench.torque.x)
        self.assertEquals(0.0, w.wrench.torque.y)
        self.assertEquals(0.0, w.wrench.torque.z)
        
        
    def test_orientation_pitch_transform(self):
        wrench_in = geometry_msgs.msg.WrenchStamped()
        angle = math.pi / 2.0
        transform = numpy.array([
            [ math.cos(angle), 0, math.sin(angle), 0],
            [               0, 1,               0, 0],
            [-math.sin(angle), 0, math.cos(angle), 0],
            [               0, 0,               0, 1]])
        
        wrench_in.wrench.force.x = 1.0
        wrench_in.wrench.force.y = 0.0
        wrench_in.wrench.force.z = 0.0
        wrench_in.wrench.torque.x = 1.0
        wrench_in.wrench.torque.y = 0.0
        wrench_in.wrench.torque.z = 0.0
        w = geometry_transformer_util.geometry_transformer_util.transform_wrench(transform, wrench_in)
        self.assertAlmostEquals(0.0, w.wrench.force.x)
        self.assertAlmostEquals(0.0, w.wrench.force.y)
        self.assertAlmostEquals(-1.0, w.wrench.force.z)
        self.assertAlmostEquals(0.0, w.wrench.torque.x)
        self.assertAlmostEquals(0.0, w.wrench.torque.y)
        self.assertAlmostEquals(-1.0, w.wrench.torque.z)
        
        
    def test_orientation_yaw_transform(self):
        wrench_in = geometry_msgs.msg.WrenchStamped()
        angle = math.pi / 2.0
        transform = numpy.array([
            [math.cos(angle), -math.sin(angle), 0, 0],
            [math.sin(angle),  math.cos(angle), 0, 0],
            [              0,                0, 1, 0],
            [              0,                0, 0, 1]])
        
        wrench_in.wrench.force.x = 1.0
        wrench_in.wrench.force.y = 0.0
        wrench_in.wrench.force.z = 0.0
        wrench_in.wrench.torque.x = 1.0
        wrench_in.wrench.torque.y = 0.0
        wrench_in.wrench.torque.z = 0.0
        w = geometry_transformer_util.geometry_transformer_util.transform_wrench(transform, wrench_in)
        self.assertAlmostEquals(0.0, w.wrench.force.x)
        self.assertAlmostEquals(1.0, w.wrench.force.y)
        self.assertAlmostEquals(0.0, w.wrench.force.z)
        self.assertAlmostEquals(0.0, w.wrench.torque.x)
        self.assertAlmostEquals(1.0, w.wrench.torque.y)
        self.assertAlmostEquals(0.0, w.wrench.torque.z)


if __name__ == '__main__':
    unittest.main()
