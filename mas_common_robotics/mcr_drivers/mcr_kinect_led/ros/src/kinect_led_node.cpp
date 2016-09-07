/*
 *  kinect_led_node.cpp
 *
 *  Created on: Aug 12, 2011
 *      Author: Frederik Hegger
 *
 */

#include <libusb.h>
#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/ColorRGBA.h>

#define MS_MAGIC_VENDOR 0x45e
#define MS_MAGIC_MOTOR_PRODUCT 0x02b0
#define GRAVITY 9.80665
#define FREENECT_COUNTS_PER_G 819.

ros::Subscriber sub_led_option;
libusb_device_handle *dev(0);

void openAuxDevice(int index = 0)
{
    libusb_device **devs;  //pointer to pointer of device, used to retrieve a list of devices
    ssize_t cnt = libusb_get_device_list(0, &devs);  //get the list of devices
    if (cnt < 0)
    {
        ROS_ERROR_STREAM("no device on USB");
        return;
    }

    int nr_mot(0);
    for (int i = 0; i < cnt; ++i)
    {
        struct libusb_device_descriptor desc;
        const int r = libusb_get_device_descriptor(devs[i], &desc);
        if (r < 0)
            continue;

        // Search for the aux
        if (desc.idVendor == MS_MAGIC_VENDOR && desc.idProduct == MS_MAGIC_MOTOR_PRODUCT)
        {
            // If the index given by the user matches our camera index
            if (nr_mot == index)
            {
                if ((libusb_open(devs[i], &dev) != 0) || (dev == 0))
                {
                    ROS_ERROR_STREAM("cannot open aux " << index);
                    return;
                }
                // Claim the aux
                libusb_claim_interface(dev, 0);
                break;
            }
            else
                nr_mot++;
        }
    }

    libusb_free_device_list(devs, 1);  // free the list, unref the devices in it
}

void setLedOption(const std_msgs::ColorRGBA light_msg)
{
    std_msgs::UInt16 optionMsg;

    if (light_msg.r == 1)
        optionMsg.data = (uint16_t) 2;
    else if (light_msg.g == 1)
        optionMsg.data = (uint16_t) 1;

    uint8_t empty[0x1];
    const uint16_t option(optionMsg.data);

    const int ret = libusb_control_transfer(dev, 0x40, 0x06, (uint16_t) option, 0x0, empty, 0x0, 0);
    if (ret != 0)
    {
        ROS_ERROR_STREAM("error in setting LED options, libusb_control_transfer returned " << ret);
        ros::shutdown();
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "mcr_kinect_led");
    ros::NodeHandle nh("~");

    int ret = libusb_init(0);
    if (ret)
    {
        ROS_ERROR_STREAM("cannot initialize libusb, error: " << ret);
        return 1;
    }

    int deviceIndex;
    nh.param<int>("device_index", deviceIndex, 0);
    openAuxDevice(deviceIndex);
    if (!dev)
    {
        ROS_ERROR_STREAM("no valid aux device found");
        libusb_exit(0);
        return 2;
    }

    sub_led_option = nh.subscribe("led_command", 1, setLedOption);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    libusb_exit(0);
    return 0;
}
