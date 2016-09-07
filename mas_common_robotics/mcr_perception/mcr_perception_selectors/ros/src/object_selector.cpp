#include <mcr_perception_selectors/object_selector.h>
#include <mcr_perception_msgs/Object.h>

#include <geometry_msgs/PoseStamped.h>

#include <limits>
#include <math.h>
#include <stdlib.h>


ObjectSelector::ObjectSelector() : nh_("~"), object_name_received_(false), object_list_received_(false), event_in_received_(false)
{
    int selection_type = 0;
    nh_.param<int>("selection_type", selection_type, BY_NAME);

    object_selection_type_ =  static_cast<SelectionType>(selection_type);

    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);
    pub_object_ = nh_.advertise<mcr_perception_msgs::Object>("output/object", 1);
    pub_object_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("output/object_pose", 1);
    pub_object_name_ = nh_.advertise<std_msgs::String>("output/object_name", 1);
    sub_object_list_ = nh_.subscribe("input/object_list", 1, &ObjectSelector::objectListCallback, this);
    sub_event_in_ = nh_.subscribe("event_in", 1, &ObjectSelector::eventCallback, this);

    ROS_INFO_STREAM("Selection type: " << object_selection_type_);

    if (object_selection_type_ == BY_NAME)
        sub_object_name_ = nh_.subscribe("input/object_name", 1, &ObjectSelector::objectNameCallback, this);
    else if (object_selection_type_ == RANDOM)
        srand(time(NULL));

    current_state_ = INIT;
}

ObjectSelector::~ObjectSelector()
{
    pub_event_out_.shutdown();
    pub_object_.shutdown();
    pub_object_pose_.shutdown();
    pub_object_name_.shutdown();
    sub_object_list_.shutdown();
    sub_event_in_.shutdown();
    sub_object_name_.shutdown();
}

void ObjectSelector::objectNameCallback(const std_msgs::String::Ptr &msg)
{
    object_name_ = *msg;
    object_name_received_ = true;
}

void ObjectSelector::objectListCallback(const mcr_perception_msgs::ObjectList::Ptr &msg)
{
    object_list_ = msg;
    object_list_received_ = true;
}

void ObjectSelector::eventCallback(const std_msgs::String::Ptr &msg)
{
    event_in_ = *msg;
    event_in_received_ = true;
}

bool ObjectSelector::selectObjectByName(mcr_perception_msgs::Object &selected_object)
{
    if (!object_name_received_)
    {
        return false;
    }

    std_msgs::String event_out;
    std::vector<mcr_perception_msgs::Object>::iterator iter;

    for (iter = object_list_->objects.begin(); iter != object_list_->objects.end(); ++iter)
    {
        // selected object is erased from list
        if (object_name_.data == iter->name)
        {
            selected_object = *iter;
            object_list_->objects.erase(iter);

            return true;
        }
    }

    // the desired object is not available
    event_out.data = "e_not_found";
    pub_event_out_.publish(event_out);

    return false;
}

bool ObjectSelector::selectRandomObject(mcr_perception_msgs::Object &selected_object)
{
    int random_object_index = rand() % object_list_->objects.size();

    selected_object = object_list_->objects[random_object_index];
    object_list_->objects.erase(object_list_->objects.begin() + random_object_index);

    return true;
}

bool ObjectSelector::selectClosestObject(mcr_perception_msgs::Object &selected_object)
{
    double current_object_distance = 0.0;
    double closest_distance = std::numeric_limits<double>::max();
    std::vector<mcr_perception_msgs::Object>::iterator iter;
    std::vector<mcr_perception_msgs::Object>::iterator closest_object;

    // find closest object
    for (iter = object_list_->objects.begin(); iter != object_list_->objects.end(); ++iter)
    {
        // calculate Euclidean distance based on x, y, z
        current_object_distance = sqrt(pow(iter->pose.pose.position.x, 2) + pow(iter->pose.pose.position.y, 2) + pow(iter->pose.pose.position.z, 2));

        if (fabs(current_object_distance) < closest_distance)
        {
            closest_distance = fabs(current_object_distance);
            closest_object = iter;
        }
    }

    // "return" selected object
    selected_object = *closest_object;

    // remove selected object from list
    object_list_->objects.erase(closest_object);

    return true;
}


void ObjectSelector::update()
{
    bool object_selected = false;
    mcr_perception_msgs::Object object;
    std_msgs::String event_out;

    // check for received events change state accordingly
    if (event_in_received_)
    {
        if (event_in_.data == "e_trigger")
            current_state_ = IDLE;

        if (event_in_.data == "e_exists")
        {
            if (!object_list_received_ || !object_list_ || object_list_->objects.empty())
            {
                event_out.data = "e_false";
            }
            else
            {
                event_out.data = "e_true";
            }
            pub_event_out_.publish(event_out);
            event_in_.data = "";
            event_in_received_ = false;
            return;
        }
        event_in_.data = "";
        event_in_received_ = false;
    }

    // only continue if state is IDLE or RUNNING
    if (current_state_ == INIT)
        return;

    // no list received
    if (!object_list_received_)
    {
        event_out.data = "e_no_object_list";
        pub_event_out_.publish(event_out);

        current_state_ = INIT;
        return;
    }

    // list is currently empty (i.e. all objects have been selected previously)
    if (object_list_->objects.empty())
    {
        event_out.data = "e_no_objects";
        pub_event_out_.publish(event_out);

        current_state_ = INIT;
        return;
    }

    if (object_selection_type_ == BY_NAME)
        object_selected = selectObjectByName(object);
    else if (object_selection_type_ == RANDOM)
        object_selected = selectRandomObject(object);
    else if (object_selection_type_ == CLOSEST)
        object_selected = selectClosestObject(object);

    if (object_selected)
    {
        pub_object_.publish(object);
        pub_object_pose_.publish(object.pose);
        std_msgs::String object_name;
        object_name.data = object.name;
        pub_object_name_.publish(object_name);

        event_out.data = "e_selected";
        pub_event_out_.publish(event_out);

        current_state_ = INIT;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_selector");
    ros::NodeHandle nh("~");

    int frame_rate = 30;    // in Hz
    ObjectSelector object_selector;

    nh.param<int>("frame_rate", frame_rate, 30);
    ROS_INFO("node started");

    ros::Rate loop_rate(frame_rate);

    while (ros::ok())
    {
        object_selector.update();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
