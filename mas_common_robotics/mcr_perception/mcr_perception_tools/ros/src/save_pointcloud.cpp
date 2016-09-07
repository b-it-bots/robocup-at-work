#include <mcr_perception_tools/save_pointcloud.h>
#include <pcl_conversions/pcl_conversions.h>

SavePointCloud::SavePointCloud() : nh_("~")
{
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);

    if (!nh_.getParam("input_topic", input_topic_))
    {
        ROS_ERROR("Input topic needs to be specified");
        exit(0);
    }

    sub_file_path_ = nh_.subscribe("input/file_path", 1, &SavePointCloud::filePathCallback, this);
    sub_pointcloud_ = nh_.subscribe(input_topic_, 1, &SavePointCloud::pointcloudCallback, this);
}

SavePointCloud::~SavePointCloud()
{
}

void SavePointCloud::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (file_path_.empty())
    {
        return;
    }
    std_msgs::String event_out;

    pcl::PCLPointCloud2 pc2;
    pcl_conversions::toPCL(*msg, pc2);

    std::string file_type;
    nh_.param<std::string>("file_type", file_type, "ascii");

    int ret = -1;
    if (file_type == "ascii")
    {
        ret = writer_.writeASCII(file_path_, pc2);
    }
    else if (file_type == "binary")
    {
        ret = writer_.writeBinary(file_path_, pc2);
    }
    else if (file_type == "compressed")
    {
        ret = writer_.writeBinaryCompressed(file_path_, pc2);
    }
    else
    {
        ROS_ERROR("Invalid file type specified %s", file_type.c_str());
    }
    if (ret == -1)
    {
        ROS_ERROR("Did not save file");
        event_out.data = "e_failed";
    }
    else {
        ROS_INFO("Saved file %s", file_path_.c_str());
        event_out.data = "e_done";
    }

    pub_event_out_.publish(event_out);
    file_path_ = "";
}

void SavePointCloud::filePathCallback(const std_msgs::String &msg)
{
    file_path_ = msg.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_pointcloud");
    ros::NodeHandle nh("~");

    int frame_rate = 5;    // in Hz
    SavePointCloud save_pointcloud;

    nh.param<int>("frame_rate", frame_rate, 5);
    ROS_INFO("node started");

    ros::Rate loop_rate(frame_rate);

    while (ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
