#!/usr/bin/env python
"""
Test unit for the functions/methods used in twist_limiter.py module.

"""

import unittest
import rosunit
import mcr_twist_limiter_ros.twist_limiter \
    as twist_limiter

PKG = 'mcr_twist_limiter'


class TestTwistLimiter(unittest.TestCase):
    """
    Tests methods used in the twist_limiter.py module.

    """
    def test_control_velocity_in_limit(self):
        """
        Tests that the 'limit_velocity' function returns the correct value,
        when the input velocity is within the velocity limits.

        """
        max_velocity_1 = 2.0
        max_velocity_2 = 1.0
        max_velocity_3 = 2.0

        velocity_1 = 0.02
        velocity_2 = -0.8
        velocity_3 = 1.96

        result_1 = 0.02
        result_2 = -0.8
        result_3 = 1.96

        self.assertAlmostEqual(
            twist_limiter.limit_velocity(velocity_1, max_velocity_1), result_1
        )
        self.assertAlmostEqual(
            twist_limiter.limit_velocity(velocity_2, max_velocity_2), result_2
        )
        self.assertAlmostEqual(
            twist_limiter.limit_velocity(velocity_3, max_velocity_3), result_3
        )

    def test_control_velocity_outside_limit(self):
        """
        Tests that the 'limit_velocity' function  returns the correct value,
        when the input velocity is outside the velocity limits.

        """
        max_velocity_1 = 2.0
        max_velocity_2 = 1.0
        max_velocity_3 = 2.0

        velocity_1 = 5.0
        velocity_2 = -8.0
        velocity_3 = 4.5

        result_1 = 2.0
        result_2 = -1.0
        result_3 = 2.0

        self.assertAlmostEqual(
            twist_limiter.limit_velocity(velocity_1, max_velocity_1), result_1
        )
        self.assertAlmostEqual(
            twist_limiter.limit_velocity(velocity_2, max_velocity_2), result_2
        )
        self.assertAlmostEqual(
            twist_limiter.limit_velocity(velocity_3, max_velocity_3), result_3
        )


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_twist_limiter', TestTwistLimiter)
