#! /usr/bin/python
import math
import sys

class p_controller:
	def __init__ (self, proportional_constant):
		if proportional_constant > 1.0 or proportional_constant < 0.0:
			sys.exit('error, proportional constant should be less than 1.0 and larger than 0.0')
		self.proportional_constant = proportional_constant
		
	def control(self, set_value, current_value):
		error = set_value - current_value
		control_value = current_value + ( self.proportional_constant * error )			
		return control_value	
		
		
class pd_controller:
	def __init__ (self, proportional_constant, derivative_constant, sampling_time):
		if proportional_constant > 1.0 or proportional_constant < 0.0:
                        sys.exit('error, proportional constant should be less than 1.0 and larger than 0.0')
                if derivative_constant > 1.0 or derivative_constant < 0.0:
                        sys.exit('error, derivative constant should be less than 1.0 and larger than 0.0')
		self.sampling_time = sampling_time
		self.error_list = [0.0] * self.sampling_time
		self.proportional_constant = proportional_constant
                self.derivative_constant = derivative_constant
		
	def control(self, set_value, current_value):
		error = set_value - current_value
		self.error_list.pop(0)
		self.error_list.append(error)
		derivative_error = self.error_list[self.sampling_time - 2] - error
		control_value = current_value + (self.proportional_constant * error) + (self.derivative_constant * derivative_error)
		return control_value

			
