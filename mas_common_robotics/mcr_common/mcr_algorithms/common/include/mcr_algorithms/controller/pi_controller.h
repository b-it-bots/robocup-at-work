/*
 * pi_controller.h
 *
 *  Created on: Mar 18, 2012
 *      Author: fred
 */

#ifndef PI_CONTROLLER_H_
#define PI_CONTROLLER_H_

#include <deque>
#include <stdlib.h>
#include <iostream>

class PIController
{
public:
	PIController(double proportional_constant, double integral_constant, double sampling_time);
	double control(double current_value, double set_value);
private:
	double proportional_constant_;
	double integral_constant_;
	double sampling_time_;
	std::deque<double> error_list_;
	unsigned int call_counter_;
};

#endif /* PO_CONTROLLER_H_ */
