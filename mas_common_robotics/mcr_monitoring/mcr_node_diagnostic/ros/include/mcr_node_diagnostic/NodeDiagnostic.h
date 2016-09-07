/*
 * NodeDiagnostic.h
 *
 *  Created on: May 23, 2013
 *      Author: matthias
 */

#ifndef NODEDIAGNOSTIC_H_
#define NODEDIAGNOSTIC_H_

#include <string.h>

#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>



namespace mcr
{



class NodeDiagnostic
{
private:

    static ros::Publisher _publisher;

    static boost::shared_ptr<ros::NodeHandle> _nodeHandle;

    static bool initialized;

    NodeDiagnostic()
    {

    }

    static void init()
    {
        _nodeHandle.reset(new ros::NodeHandle());

        _publisher = _nodeHandle->advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 100);

        initialized = true;
        //wait for initialization of the _publisher
        usleep(500000);

    }


public:

    enum Level
    {
        OK,
        WARN,
        ERROR
    };

    enum Category
    {
        SOFTWARE,
        HARDWARE,
        COMMUNICATION,
        INTERACTION ,
        ALGORITHMIC,
        DEFAULT
    };

    static const char* CategoryNames[];

    virtual ~NodeDiagnostic() { }

    static void _diag(std::string ID, Level level, Category category, std::string msg, std::vector<diagnostic_msgs::KeyValue> keyValue)
    {
        if (!initialized)
        {
            NodeDiagnostic::init();
        }

        diagnostic_msgs::DiagnosticArray rosmsg;
        rosmsg.header.stamp = ros::Time::now();

        diagnostic_msgs::DiagnosticStatus status;
        status.level = level;
        std::string fullname = "mcr_diag/" + ros::this_node::getNamespace() + "/" + ros::this_node::getName() + "/" + ID + "/";

        //remove double //
        while (fullname.find("//") != std::string::npos)
        {
            fullname = fullname.erase(fullname.find("//"), 1);
        }
        status.name = fullname;
        status.message = msg;
        status.values = keyValue;

        diagnostic_msgs::KeyValue kv;
        kv.key = "failure_category";
        kv.value = CategoryNames[category];
        status.values.push_back(kv);

        rosmsg.status.push_back(status);

        NodeDiagnostic::_publisher.publish(rosmsg);


    }

    static void warn(std::string ID, Category category, std::string msg, std::vector<diagnostic_msgs::KeyValue> keyValue)
    {
        _diag(ID, WARN, category, msg, keyValue);
    }

    static void warn(std::string ID, Category category, std::string msg)
    {
        std::vector<diagnostic_msgs::KeyValue> keyValue;
        _diag(ID, WARN, category, msg, keyValue);
    }

    static void error(std::string ID, Category category, std::string msg, std::vector<diagnostic_msgs::KeyValue> keyValue)
    {
        _diag(ID, ERROR, category, msg, keyValue);
    }

    static void error(std::string ID, Category category, std::string msg)
    {
        std::vector<diagnostic_msgs::KeyValue> keyValue;
        _diag(ID, ERROR, category, msg, keyValue);
    }

    static void ok(std::string ID, Category category, std::string msg, std::vector<diagnostic_msgs::KeyValue> keyValue)
    {
        _diag(ID, OK, category, msg, keyValue);
    }

    static void ok(std::string ID, Category category, std::string msg)
    {
        std::vector<diagnostic_msgs::KeyValue> keyValue;
        _diag(ID, OK, category, msg, keyValue);
    }

    static void reset(std::string ID)
    {
        std::vector<diagnostic_msgs::KeyValue> keyValue;
        _diag(ID, OK, DEFAULT, "", keyValue);
    }

    static diagnostic_msgs::KeyValue keyValue(std::string key, std::string value)
    {
        diagnostic_msgs::KeyValue kv;
        kv.key = key;
        kv.value = value;
        return kv;
    }
};


// init variables
boost::shared_ptr<ros::NodeHandle> NodeDiagnostic::_nodeHandle;
ros::Publisher NodeDiagnostic::_publisher;
bool NodeDiagnostic::initialized = false;

const char* NodeDiagnostic::CategoryNames[] =
{
    "software",
    "hardware",
    "communication",
    "interaction",
    "algorithmic",
    "default"
};


} /* namespace mcr */
#endif /* NODEDIAGNOSTIC_H_ */
