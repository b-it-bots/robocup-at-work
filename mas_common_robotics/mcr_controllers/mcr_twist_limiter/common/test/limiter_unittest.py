#!/usr/bin/env python
"""
Test unit for the functions in the limiter.py module.

To run it, type:
nosetests [file_name]

"""
import nose.tools
import mcr_twist_limiter.limiter


def test_control_velocity_in_limit():
    """
    Tests that the 'limit' function returns the correct value,
    when the input value does not exceed its limit.

    """
    nose.tools.assert_equal(mcr_twist_limiter.limiter.limit_value(0.02, 2.0), 0.02)
    nose.tools.assert_equal(mcr_twist_limiter.limiter.limit_value(-0.8, 1.0), -0.8)
    nose.tools.assert_equal(mcr_twist_limiter.limiter.limit_value(1.96, 2.0), 1.96)


def test_control_velocity_outside_limit():
    """
    Tests that the 'limit' function returns the correct value,
    when the input value does exceed its limit.

    """
    nose.tools.assert_equal(mcr_twist_limiter.limiter.limit_value(3.02, 2.0), 2.0)
    nose.tools.assert_equal(mcr_twist_limiter.limiter.limit_value(-1.8, 1.0), -1.0)
    nose.tools.assert_equal(mcr_twist_limiter.limiter.limit_value(2.96, 2.0), 2.0)
