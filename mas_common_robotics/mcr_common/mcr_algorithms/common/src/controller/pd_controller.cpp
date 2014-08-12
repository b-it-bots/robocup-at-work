/*
 * pd_controller.cpp
 *
 *  Created on: Mar 18, 2012
 *      Author: fred
 */

#include "mcr_algorithms/controller/pd_controller.h"

PDController::PDController(double proportional_constant, double derivative_constant, double sampling_time)
{
	if(proportional_constant > 1.0 or proportional_constant < 0.0)
	{
		std::cerr << "proportional constant should be less than 1.0 and larger than 0.0";
		exit(0);
	}
	if(derivative_constant > 1.0 or derivative_constant < 0.0)
	{
		std::cerr << "derivative constant should be less than 1.0 and larger than 0.0";
		exit(0);
	}

	this->proportional_constant_ = proportional_constant;
	this->derivative_constant_ = derivative_constant;
	this->sampling_time_ = sampling_time;

	for(unsigned int i=0; i < this->sampling_time_; ++i)
		this->error_list_.push_back(0.0f);
}

double PDController::control(double current_value, double set_value)
{
	double error = 0, control_value = 0, derivative_error = 0;

	error = set_value - current_value;
	this->error_list_.pop_front();
	this->error_list_.push_back(error);

	derivative_error = this->error_list_[this->sampling_time_ - 2] - error;

	control_value = current_value + (this->proportional_constant_ * error) + (this->derivative_constant_ * derivative_error);

	return control_value;
}


