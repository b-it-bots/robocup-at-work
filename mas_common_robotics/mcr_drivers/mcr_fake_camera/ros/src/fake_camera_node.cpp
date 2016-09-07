#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <ros/ros.h>


int main(int argc, char** argv)
{
    if (argc != 2)
        std::cout << "Please provide an image file as argument" << std::endl;

    ros::init(argc, argv, "mcr_fake_camera");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/stereo/left/image_mono", 1);

    cv::WImageBuffer1_b image(cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE));

    cv_bridge::CvImage out_msg;
    out_msg.image = image.Ipl();

    ros::Rate loop_rate(10);
    while (nh.ok())
    {
        out_msg.header.stamp = ros::Time::now();
        pub.publish(out_msg.toImageMsg());
        ros::spinOnce();
        loop_rate.sleep();
    }
}

