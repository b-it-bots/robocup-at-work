/*
 * pd_controller.h
 *
 *  Created on: Mar 18, 2012
 *      Author: fred
 */

#ifndef PD_CONTROLLER_H_
#define PD_CONTROLLER_H_

#include <deque>
#include <stdlib.h>
#include <iostream>

class PDController
{
public:
    PDController(double proportional_constant, double derivative_constant, double sampling_time);
    double control(double current_value, double set_value);
private:
    double proportional_constant_;
    double derivative_constant_;
    double sampling_time_;
    std::deque<double> error_list_;
};

#endif /* PD_CONTROLLER_H_ */
