#ifndef OBJECT_SELECTOR_H_
#define OBJECT_SELECTOR_H_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <mcr_perception_msgs/ObjectList.h>

/**
 * This class selects the specified object from a list of perceived objects.
 *
 * When a new list of objects is published (by a perception node), the current list
 * is discarded and replaced by the new one.
 *
 * When an object name is specified, the first instance of the object in the current list
 * is selected and its pose is published. Events are generated if no lists exists, the object does
 * not exist etc. Once an object is selected, it is removed from the list.
 *
 * Subscribes:
 *  ~/input/object_list - list of perceived objects
 *  ~/input/object_name - object to be selected
 *
 *  Publishes:
 *  ~/output/pose - PoseStamped of the selected object
 *  ~/event_out - "e_no_object_list" - no list is available
 *                "e_no_objects" - list is empty
 *                "e_selected" - object selected
 *                "e_not_found" - object not found in list
 */

class ObjectSelector
{
public:
    ObjectSelector();
    virtual ~ObjectSelector();

    void update();

    enum SelectionType {BY_NAME = 0, RANDOM = 1, CLOSEST = 2};
    enum States {INIT, IDLE, RUNNING};

private:
    void objectNameCallback(const std_msgs::String::Ptr &msg);
    void objectListCallback(const mcr_perception_msgs::ObjectList::Ptr &msg);
    void eventCallback(const std_msgs::String::Ptr &msg);

    bool selectObjectByName(mcr_perception_msgs::Object &selected_object);
    bool selectRandomObject(mcr_perception_msgs::Object &selected_object);
    bool selectClosestObject(mcr_perception_msgs::Object &selected_object);

private:
    ros::NodeHandle nh_;

    States current_state_;

    ros::Subscriber sub_object_name_;
    ros::Subscriber sub_object_list_;
    ros::Subscriber sub_event_in_;

    ros::Publisher pub_event_out_;
    ros::Publisher pub_object_;
    ros::Publisher pub_object_pose_;
    ros::Publisher pub_object_name_;

    std_msgs::String object_name_;
    bool object_name_received_;

    mcr_perception_msgs::ObjectList::Ptr object_list_;
    bool object_list_received_;

    std_msgs::String event_in_;
    bool event_in_received_;

    SelectionType object_selection_type_;

};
#endif
