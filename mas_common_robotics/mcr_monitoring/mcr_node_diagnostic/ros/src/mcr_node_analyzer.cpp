/*
 * mcr_node_diagnostic.cpp
 *
 *  Created on: May 16, 2013
 *      Author: matthias fueller (matthias.fueller@h-brs.de)
 */

#include "mcr_node_diagnostic/analyser/mcr_node_analyzer.h"

#include <boost/algorithm/string.hpp>

using namespace std;

PLUGINLIB_REGISTER_CLASS(mcr_node_analyzer,
                         diagnostic_aggregator::mcr_node_analyzer,
                         diagnostic_aggregator::Analyzer)

namespace diagnostic_aggregator
{

mcr_node_analyzer::mcr_node_analyzer() :
    path_("/mcr_node_analyzer"), nice_name_("mcr_diagnostic"), prefix(
        "mcr_diag")

{
}

mcr_node_analyzer::~mcr_node_analyzer()
{
}

bool mcr_node_analyzer::init(const string base_name,
                             const ros::NodeHandle &n)
{

    // nothing to init

    return true;
}

bool mcr_node_analyzer::match(const std::string name)
{

    if ((name.substr(0, prefix.size()) == prefix))
        return true;

    return false;
}

bool mcr_node_analyzer::analyze(const boost::shared_ptr<StatusItem> item)
{
    bool new_item = true;

    for (size_t i = 0; i < items.size(); i++)
    {

        if (items[i]->getName() == item->getName())
        {
            items[i] = item;
            new_item = false;
            break;
        }

    }

    if (new_item)
    {
        items.push_back(item);
    }

    std::vector<boost::shared_ptr<StatusItem> > tmp_items;

    for (size_t i = 0; i < items.size(); i++)
    {
        if (items[i]->getLastUpdateTime() < ros::Time::now() + ros::Duration(2)
                && items[i]->getLevel() == 0)
        {
            //skip item
        }
        else
        {
            tmp_items.push_back(items[i]);
        }

    }

    items = tmp_items;

    return true;
}

vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > mcr_node_analyzer::report()
{

    vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > output;

    for (size_t i = 0; i < items.size(); i++)
    {

        if (items[i]->getLastUpdateTime() < ros::Time::now() + ros::Duration(5)
                && items[i]->getLevel() == 0)
        {
            continue;
        }

        boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> stat(
            new diagnostic_msgs::DiagnosticStatus());

        stat = items[i]->toStatusMsg("", false);
        stat->name = items[i]->getName();

        output.push_back(stat);

        // Code for setting the complete branch to WARN / ERROR
        /*
         std::string pathname = "";

         if (stat->level > 0) {
         vector<string> strs;
         boost::split(strs,stat->name,boost::is_any_of("/"));

         for(size_t i=0; i<strs.size() - 1; i++) {
         pathname += "/" + strs[i];
         boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> s(new diagnostic_msgs::DiagnosticStatus());

         s->level = stat->level;
         s->name = pathname;

         output.push_back(s);
         }

         */

    }

    return output;
}

} // end of namespace diagnostic_aggregator

