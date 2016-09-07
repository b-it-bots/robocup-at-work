/* 
 * Copyright [2016] <Bonn-Rhein-Sieg University>  
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * RunScript class, calls system() function 
 * for calling external bash scripts from c++ code
 * 
 */

#include <mcr_task_planning_tools/run_script.h>
#include <string>
#include <vector>

RunScript::RunScript() : full_path_to_script_(""), script_arguments_(""),
is_script_path_set_(false), are_args_available_(false) {}

bool RunScript::run(std::string &full_path_to_script)
{
    if (system(full_path_to_script_.c_str()) == 0)
    {
        return true;
    }

    return false;
}

void RunScript::setScriptPath(std::string &full_path_to_script)
{
    full_path_to_script_ = full_path_to_script;
    is_script_path_set_ = true;
}

void RunScript::setScriptArgs(std::vector<std::string> &script_arguments)
{
    are_args_available_ = true;

    for (int i=0; i < script_arguments.size(); i++)
    {
        script_arguments_ += std::string(" ");
        script_arguments_ += script_arguments.at(i);
    }
}

bool RunScript::run()
{
    if (is_script_path_set_)
    {
        if (are_args_available_)
        {
            if (system((full_path_to_script_ + script_arguments_).c_str()) == 0)
            {
                return true;
            }

            return false;
        }
        else
        {
            if (system(full_path_to_script_.c_str()) == 0)
            {
                return true;
            }

            return false;
        }
    }

    return false;
}
