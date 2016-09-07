/*
 * pi_controller.cpp
 *
 *  Created on: Mar 18, 2012
 *      Author: fred
 */

#include "mcr_algorithms/controller/pi_controller.h"

PIController::PIController(double proportional_constant, double integral_constant, double sampling_time)
{
    if (proportional_constant > 1.0 or proportional_constant < 0.0)
    {
        std::cerr << "proportional constant should be less than 1.0 and larger than 0.0";
        exit(0);
    }
    if (integral_constant > 1.0 or integral_constant < 0.0)
    {
        std::cerr << "derivative constant should be less than 1.0 and larger than 0.0";
        exit(0);
    }

    this->proportional_constant_ = proportional_constant;
    this->integral_constant_ = integral_constant;
    this->sampling_time_ = sampling_time;
    this->call_counter_ = 0;

    for (unsigned int i = 0; i < this->sampling_time_; ++i)
        this->error_list_.push_back(0.0f);
}

double PIController::control(double current_value, double set_value)
{
    double error = 0, control_value = 0;

    error = set_value - current_value;
    this->error_list_.pop_front();
    this->error_list_.push_back(error);


    if (this->call_counter_ < this->sampling_time_)
        ++this->call_counter_;

    double sum_error = 0;
    for (unsigned int i = 0; i < this->error_list_.size(); ++i)
        sum_error += this->error_list_[i];

    control_value = current_value + (this->proportional_constant_ * error) + (this->integral_constant_ * sum_error);

//  std::cout << "current_value: " << current_value << " set_value: " << set_value << " sum_error: " << sum_error << " control_value: " << control_value << std::endl;

    return control_value;
}


