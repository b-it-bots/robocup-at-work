#!/usr/bin/env python
"""
Test unit for the functions/methods used in distance_constrained.py module.

"""

PKG = 'mcr_guarded_approach_pose'

import unittest
import rosunit
import mcr_guarded_approach_pose_ros.distance_constrained as distance_constrained


class TestDistanceConstrained(unittest.TestCase):
    """
    Tests methods used in the distance_constrained.py module.

    """
    def test_string_in_list_element_no_matches(self):
        """
        Tests that the 'string_in_list_element' function returns None, since
        no matches should be found.

        """
        list_of_strings = ['aaaa', 'bbbb', 'cccc', 'dddd']
        string = 'x'

        actual = distance_constrained.string_in_list_element(string, list_of_strings)

        self.assertIsNone(actual)

    def test_string_in_list_element_one_string(self):
        """
        Tests that the 'string_in_list_element' function returns the correct string.

        """
        list_of_strings = ['aaaa', 'bbbb', 'cccc', 'dddd']
        string = 'b'

        # 'b' is in 'bbbb'
        desired = list_of_strings[1]
        actual = distance_constrained.string_in_list_element(string, list_of_strings)

        self.assertEqual(actual, desired)

    def test_string_in_list_element_many_strings(self):
        """
        Tests that the 'string_in_list_element' function returns only one string,
        even if multiple matches are found.

        """
        list_of_strings = ['abaa', 'bbbb', 'cccc', 'dddd']
        string = 'b'

        # 'b' is in 'aaaa' and 'bbbb', but 'aaaa' is first
        desired = list_of_strings[0]
        actual = distance_constrained.string_in_list_element(string, list_of_strings)

        self.assertEqual(actual, desired)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_distance_constrained', TestDistanceConstrained)
