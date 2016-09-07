/*
 * mcr_node_diagnostic.h
 *
 *  Created on: May 16, 2013
 *      Author: matthias fueller (matthias.fueller@h-brs.de)
 */

#ifndef mcr_NODE_DIAGNOSTIC_H_
#define mcr_NODE_DIAGNOSTIC_H_

#include <ros/ros.h>
#include <diagnostic_aggregator/analyzer.h>
#include <diagnostic_aggregator/status_item.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pluginlib/class_list_macros.h>
#include <string>

namespace diagnostic_aggregator
{

class mcr_node_analyzer: public Analyzer
{
public:
    mcr_node_analyzer();

    ~mcr_node_analyzer();

    bool init(const std::string base_name, const ros::NodeHandle &n);

    bool match(const std::string name);

    bool analyze(const boost::shared_ptr<StatusItem> item);

    std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > report();

    std::string getPath() const
    {
        return path_;
    }

    std::string getName() const
    {
        return nice_name_;
    }

private:

    std::vector<boost::shared_ptr<StatusItem> > items;

    std::string path_, nice_name_;

    std::string prefix;

};

}

#endif /* mcr_NODE_DIAGNOSTIC_H_ */
