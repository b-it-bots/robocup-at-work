#ifndef CAVITY_TEMPLATE_PUBLISHER_ROS_H_
#define CAVITY_TEMPLATE_PUBLISHER_ROS_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;


/**
 * This class selects the correct cavity contour template for a given object
 * It reads the preferred_orientation param to select a vertical or horizontal placement cavity
 *
 * Subscribes to
 * - input/object_category: string specifying object name to find a cavity for
 *
 * Publishes:
 * - output/template_pointcloud: a pointcloud for the chosen cavity contour
 *
 */
class CavityTemplatePublisherROS
{
public:
    /**
     * Constructor
     * Sets template_path_ and initializes publisher and subscriber
     */
    CavityTemplatePublisherROS();
    /**
     * Destructor
     */
    virtual ~CavityTemplatePublisherROS();


private:
    /**
     * Copy constructor.
     */
    CavityTemplatePublisherROS(const CavityTemplatePublisherROS &other);

    /**
     * Copy assignment operator.
     */
    CavityTemplatePublisherROS &operator=(CavityTemplatePublisherROS other);

    /**
     * Callback to receive object name for which to find a template
     */
    void objectNameCallback(const std_msgs::StringPtr &msg);

    /**
     * Helper function to map object name to template filename.
     * Uses param preferred_orientation to select horizontal or vertical
     * template.
     *
     * @param object_name
     * string specifying object name
     *
     * @return filename of chosen template
     */
    std::string getTemplateFilename(const std::string &object_name);


private:
    /**
     * Node handle
     */
    ros::NodeHandle nh_;

    /**
     * Subscriber for object name
     */
    ros::Subscriber sub_object_name_;

    /**
     * Publisher for template pointcloud
     */
    ros::Publisher pub_template_pointcloud_;

    /**
     * Path to template pointcloud
     */
    bfs::path template_path_;
};
#endif
