#! /usr/bin/env python
import math

'''
the file contain function for moving average low pass filter
'''

class moving_average:
	def __init__(self, number_of_points):
		self.number_of_points = number_of_points
		self.points_list = [0.0] * self.number_of_points

	def add_points(self, value):
		self.points_list.pop(0)
		self.points_list.append(value)	
	
	def value(self):
		value = math.fsum(self.points_list) / self.number_of_points
		return  value
