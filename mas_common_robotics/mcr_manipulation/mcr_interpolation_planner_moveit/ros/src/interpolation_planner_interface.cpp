/*********************************************************************
 * Software License Agreement (GPLv3 License)
 *
 *  Copyright (c) 2015, Hochschule Bonn-Rhein-Sieg.
 *  All rights reserved.
 *
 *********************************************************************/
/**
 * Author: Shehzad Ahmed
 */

#include <mcr_interpolation_planner_moveit/interpolation_planner_interface.h>


interpolation_planner_interface::InterpolationPlannerInterface::InterpolationPlannerInterface(
    const robot_model::RobotModelConstPtr &kmodel,
    const ros::NodeHandle &nh) :
    nh_(nh),
    kmodel_(kmodel)
{
    logInform("Initializing Interpolation planner interface using ROS parameters");
    loadPlannerConfigurations();
    loadGroupConfigurations();
}

interpolation_planner_interface::InterpolationPlannerInterface::~InterpolationPlannerInterface()
{
}

void interpolation_planner_interface::InterpolationPlannerInterface::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap &pconfig)
{
    planning_interface::PlannerConfigurationMap pconfig2 = pconfig;

    /**
     * Construct default configurations for planning groups
     * that don't have configs already passed in.
     */
    const std::vector<const robot_model::JointModelGroup*>& groups = kmodel_->getJointModelGroups();

    for (std::size_t i = 0 ; i < groups.size() ; ++i)
    {
        if (pconfig.find(groups[i]->getName()) == pconfig.end())
        {
            planning_interface::PlannerConfigurationSettings empty;
            empty.name = empty.group = groups[i]->getName();
            pconfig2[empty.name] = empty;
        }
    }

    planner_configs_ = pconfig2;
}

interpolation_planner_interface::InterpolationPlannerContextPtr
interpolation_planner_interface::InterpolationPlannerInterface::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest &req,
    moveit_msgs::MoveItErrorCodes &error_code) const
{
    if (req.group_name.empty())
    {
        logError("No kinematic group is supplied in planning request.");
        error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
        return InterpolationPlannerContextPtr();
    }

    if (req.goal_constraints.size() == 0)
    {
        logError("No goal constraint is supplied in planning request.");
        return InterpolationPlannerContextPtr();
    }

    error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;

    if (!planning_scene)
    {
        logError("No planning scene supplied in planning request.");
        return InterpolationPlannerContextPtr();
    }

    /**
     * Identify the correct planning configuration.
     */
    planning_interface::PlannerConfigurationMap::const_iterator pc = planner_configs_.end();

    if (pc == planner_configs_.end())
    {
        pc = planner_configs_.find(req.group_name);
        if (pc == planner_configs_.end())
        {
            logError("Cannot find planning configuration for group '%s'", req.group_name.c_str());
            return InterpolationPlannerContextPtr();
        }
    }

    planning_interface::PlannerConfigurationSettings pc_temp = pc->second;

    if (req.planner_id.empty())
    {
        logWarn("No planner is specified in request. Using default planner %s", "Trapezoidal");
        pc_temp.config["selected_planner_id"] = "Trapezoidal";
    }
    else
    {
        int number_of_planners = boost::lexical_cast<int>(pc_temp.config["number_of_planners"]);
        bool is_planner_type_exit = false;

        for (int iter = 1; iter <= number_of_planners; iter++)
        {
            std::string planner_type = pc_temp.config["planner_type_" + iter];
            if (req.planner_id.compare(planner_type) == 0)
                is_planner_type_exit = true;
        }

        if (!is_planner_type_exit)
        {
            logError("Planner type '%s' does not exit in the planner configurations.", req.planner_id.c_str());
            return InterpolationPlannerContextPtr();
        }

        pc_temp.config["selected_planner_id"] = req.planner_id;
    }

    logInform("selected_planner_id_: %s", pc_temp.config["selected_planner_id"].c_str());

    interpolation_planner_interface::GroupSpecificationMap::const_iterator gc = group_configs_.end();

    if (gc == group_configs_.end())
    {
        gc = group_configs_.find(req.group_name);
        if (gc == group_configs_.end())
        {
            logError("Cannot find planning configuration for group '%s'", req.group_name.c_str());
            return InterpolationPlannerContextPtr();
        }
    }

    InterpolationPlannerContextPtr context(new InterpolationPlannerContext(req.group_name,
                                           pc_temp,
                                           gc->second,
                                           kmodel_));

    if (context)
    {
        context->clear();

        robot_state::RobotStatePtr start_state = planning_scene->getCurrentStateUpdated(req.start_state);

        /**
         * Setup the context.
         */
        context->setPlanningScene(planning_scene);
        context->setMotionPlanRequest(req);
        context->setCompleteInitialState(*start_state);

        if (req.goal_constraints.size() > 1)
        {
            logDebug("Planning request specified more than one goal constraints."
                     "Current implementation of the interpolation planner"
                     "plans for only first goal constraint.");
        }

        // Ignore more then one goal constraints.
        if (!context->setGoalConstraints(req.goal_constraints[0], &error_code))
            return InterpolationPlannerContextPtr();

        logDebug("%s: New planning context is set.", context->getName().c_str());
        error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }

    return context;
}

void interpolation_planner_interface::InterpolationPlannerInterface::loadPlannerConfigurations()
{
    const std::vector<std::string> &group_names = kmodel_->getJointModelGroupNames();
    planning_interface::PlannerConfigurationMap pconfig;

    /**
     * The set of planning parameters that can be specific for
     * the group (inherited by configurations of that group).
     */
    static const std::string KNOWN_GROUP_PARAMS[] =
    {
        "planner_minimum_sample_time"
    };

    /**
     * Read the planning configuration for each group.
     */
    pconfig.clear();
    for (std::size_t i = 0 ; i < group_names.size() ; ++i)
    {
        /**
         * Get parameters specific for the robot planning group.
         */
        std::map<std::string, std::string> specific_group_params;

        for (std::size_t k = 0 ; k < sizeof(KNOWN_GROUP_PARAMS) / sizeof(std::string) ; ++k)
        {
            if (nh_.hasParam(group_names[i] + "/" + KNOWN_GROUP_PARAMS[k]))
            {
                double value_d;
                if (nh_.getParam(group_names[i] + "/" + KNOWN_GROUP_PARAMS[k], value_d))
                    specific_group_params[KNOWN_GROUP_PARAMS[k]] = boost::lexical_cast<std::string>(value_d);
                else
                {
                    logError("Parameter '%s' should be of type double(for group '%s')",
                             KNOWN_GROUP_PARAMS[k].c_str(), group_names[i].c_str());
                }
            }
            else
            {
                logError("Could not find the parameter '%s' on the param server(for group '%s')",
                         KNOWN_GROUP_PARAMS[k].c_str(), group_names[i].c_str());
            }
        }

        /**
         * Get parameters specific to each planner type.
         */
        XmlRpc::XmlRpcValue config_names;
        if (!nh_.getParam(group_names[i] + "/planner_configs", config_names))
        {
            logError("Could not find Planner configurations on parameter server"
                     " for group '%s'", group_names[i].c_str());
            continue;
        }

        if (!(config_names.getType() == XmlRpc::XmlRpcValue::TypeArray))
        {
            logError("The planner_configs argument of a group configuration "
                     "should be an array of strings (for group '%s')", group_names[i].c_str());
            continue;
        }

        for (int32_t j = 0; j < config_names.size() ; ++j)
        {
            std::string planner_config;
            if (!(config_names[j].getType() == XmlRpc::XmlRpcValue::TypeString))
            {
                logError("Planner configuration names must be of "
                         "type string (for group '%s')", group_names[i].c_str());
                continue;
            }

            planner_config = static_cast<std::string>(config_names[j]);

            XmlRpc::XmlRpcValue xml_config;
            if (!nh_.getParam("planner_configs/" + planner_config, xml_config))
            {
                logError("Could not find the planner configuration '%s' "
                         "on the param server", planner_config.c_str());
                continue;
            }

            if (!(xml_config.getType() == XmlRpc::XmlRpcValue::TypeStruct))
            {
                logError("A planning configuration should be of type XmlRpc "
                         "Struct type (for configuration '%s')", planner_config.c_str());
            }
            else
            {
                planning_interface::PlannerConfigurationSettings pc;
                pc.name = group_names[i];
                pc.group = group_names[i];
                pc.config = specific_group_params;

                /**
                 * Read planner type.
                 */
                int number_of_planners = 0;
                for (XmlRpc::XmlRpcValue::iterator it = xml_config.begin() ; it != xml_config.end() ; ++it)
                    if (it->second.getType() == XmlRpc::XmlRpcValue::TypeString)
                    {
                        number_of_planners++;
                        std::string planner_type = "planner_type_" + number_of_planners;
                        pc.config[planner_type] = static_cast<std::string>(it->second);
                    }
                    else
                    {
                        logError("Planner type should be of type XmlRpc String type "
                                 "(for configuration '%s')", planner_config.c_str());
                    }

                pc.config["number_of_planners"] = boost::to_string(number_of_planners);
                pconfig[pc.name] = pc;
            }
        }
    }

    for (planning_interface::PlannerConfigurationMap::iterator it = pconfig.begin();
            it != pconfig.end(); ++it)
    {
        ROS_DEBUG_STREAM_NAMED("parameters", "Parameters for configuration '" << it->first << "'");
        for (std::map<std::string, std::string>::const_iterator config_it = it->second.config.begin() ;
                config_it != it->second.config.end() ; ++config_it)
            ROS_DEBUG_STREAM_NAMED("parameters", " - " << config_it->first << " = " << config_it->second);
    }
    setPlannerConfigurations(pconfig);
}

void interpolation_planner_interface::InterpolationPlannerInterface::loadGroupConfigurations()
{
    const std::vector<std::string> &group_names = kmodel_->getJointModelGroupNames();
    interpolation_planner_interface::GroupSpecificationMap gconfig;

    /**
     * Read the group configuration for each group.
     */
    gconfig.clear();
    for (std::size_t g_index = 0 ; g_index < group_names.size() ; ++g_index)
    {
        interpolation_planner_interface::GroupSpecification gspec;
        interpolation_planner_interface::JointSpecification jspec;
        const std::vector<std::string> &joint_names =
            kmodel_->getJointModelGroup(group_names[g_index])->getActiveJointModelNames();
        gspec.joint_names = joint_names;

        for (std::size_t j_index = 0 ; j_index < joint_names.size() ; ++j_index)
        {
            const moveit::core::JointModel *joint_model =  kmodel_->getJointModel(joint_names[j_index]);
            const moveit::core::VariableBounds &v_bounds = kmodel_->getVariableBounds(joint_names[j_index]);

            jspec.acceleration_bounded_ = v_bounds.acceleration_bounded_;
            jspec.max_acceleration_ = v_bounds.max_acceleration_;
            jspec.min_acceleration_ = v_bounds.min_acceleration_;

            jspec.max_position_ = v_bounds.max_position_;
            jspec.min_position_ = v_bounds.min_position_;
            jspec.position_bounded_ = v_bounds.position_bounded_;

            jspec.max_velocity_ = v_bounds.max_velocity_;
            jspec.min_velocity_ = v_bounds.min_velocity_;
            jspec.velocity_bounded_ = v_bounds.velocity_bounded_;

            gspec.joint_spec_map[joint_names[j_index]] = jspec;
        }

        gconfig[group_names[g_index]] = gspec;
    }

    group_configs_ = gconfig;
}

void interpolation_planner_interface::InterpolationPlannerInterface::printStatus()
{
    logInform("Interpolation planner interface is running.");
}
