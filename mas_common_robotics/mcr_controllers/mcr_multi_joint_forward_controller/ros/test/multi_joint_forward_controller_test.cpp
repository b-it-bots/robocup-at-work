#include <mcr_multi_joint_forward_controller/multi_joint_forward_controller.h>
#include <brics_actuator/JointVelocities.h>
#include <gtest/gtest.h>

using namespace forward_command_controller;


TEST(multi_joint_forward_controller_test, test_construction)
{
    MultiJointForwardCommandController < brics_actuator::JointVelocities,
                                       hardware_interface::VelocityJointInterface > c;
}

TEST(multi_joint_forward_controller_test, test_init_with_missing_joints)
{
    ros::NodeHandle nh;
    hardware_interface::VelocityJointInterface hw;
    MultiJointForwardCommandController < brics_actuator::JointVelocities,
                                       hardware_interface::VelocityJointInterface > c;

    double val;
    hardware_interface::JointStateHandle state_handle("arm_1_joint", &val, &val, &val);
    hardware_interface::JointHandle handle(state_handle, &val);
    hw.registerHandle(handle);

    ASSERT_FALSE(c.init(&hw, nh, nh));
}

TEST(multi_joint_forward_controller_test, test_init_with_all_joints)
{
    ros::NodeHandle nh;
    hardware_interface::VelocityJointInterface hw;
    MultiJointForwardCommandController < brics_actuator::JointVelocities,
                                       hardware_interface::VelocityJointInterface > c;

    double val;
    hardware_interface::JointStateHandle state_handle_1("arm_1_joint", &val, &val, &val);
    hardware_interface::JointHandle handle_1(state_handle_1, &val);
    hw.registerHandle(handle_1);
    hardware_interface::JointStateHandle state_handle_2("arm_2_joint", &val, &val, &val);
    hardware_interface::JointHandle handle_2(state_handle_2, &val);
    hw.registerHandle(handle_2);

    ASSERT_TRUE(c.init(&hw, nh, nh));
}

TEST(multi_joint_forward_controller_test, test_sending_command)
{
    ros::NodeHandle nh;
    hardware_interface::VelocityJointInterface hw;
    MultiJointForwardCommandController < brics_actuator::JointVelocities,
                                       hardware_interface::VelocityJointInterface > c;

    double cmd1;
    double val1;
    hardware_interface::JointStateHandle state_handle_1("arm_1_joint", &val1, &val1, &val1);
    hardware_interface::JointHandle handle_1(state_handle_1, &cmd1);
    hw.registerHandle(handle_1);
    double cmd2;
    double val2;
    hardware_interface::JointStateHandle state_handle_2("arm_2_joint", &val2, &val2, &val2);
    hardware_interface::JointHandle handle_2(state_handle_2, &cmd2);
    hw.registerHandle(handle_2);

    ros::Time time;
    ros::Duration duration;

    ASSERT_TRUE(c.init(&hw, nh, nh));
    c.starting(time);

    ros::Publisher pub = nh.advertise<brics_actuator::JointVelocities>("command_vel", 1, true);

    brics_actuator::JointVelocities vel;
    vel.velocities.resize(2);
    vel.velocities[0].value = 42.5;
    vel.velocities[1].value = 43.5;
    double accuracy = 0.0001;

    // it takes some time for the message to be sent and received
    while(true)
    {
        pub.publish(vel);
        c.update(time, duration);
        ros::spinOnce();
               
        if( (fabs(handle_1.getCommand() - vel.velocities[0].value) < accuracy) && (fabs(handle_2.getCommand() - vel.velocities[1].value) < accuracy) )
            break;
    }

    ASSERT_NEAR(handle_1.getCommand(), vel.velocities[0].value, accuracy);
    ASSERT_NEAR(handle_2.getCommand(), vel.velocities[1].value, accuracy);
}

TEST(multi_joint_forward_controller_test, test_sending_too_many_joints)
{

    ros::NodeHandle nh;
    hardware_interface::VelocityJointInterface hw;
    MultiJointForwardCommandController < brics_actuator::JointVelocities,
                                       hardware_interface::VelocityJointInterface > c;

    double cmd1 = 0.0;
    double val1;
    hardware_interface::JointStateHandle state_handle_1("arm_1_joint", &val1, &val1, &val1);
    hardware_interface::JointHandle handle_1(state_handle_1, &cmd1);
    hw.registerHandle(handle_1);
    double cmd2 = 0.0;
    double val2;
    hardware_interface::JointStateHandle state_handle_2("arm_2_joint", &val2, &val2, &val2);
    hardware_interface::JointHandle handle_2(state_handle_2, &cmd2);
    hw.registerHandle(handle_2);

    ros::Time time;
    ros::Duration duration;

    ASSERT_TRUE(c.init(&hw, nh, nh));
    c.starting(time);

    ros::Publisher pub = nh.advertise<brics_actuator::JointVelocities>("command_vel", 1);

    brics_actuator::JointVelocities vel;
    vel.velocities.resize(4);
    vel.velocities[0].value = 42.5;
    vel.velocities[1].value = 43.5;
    vel.velocities[2].value = 44.5;
    vel.velocities[3].value = 45.5;

    // it takes some time for the message to be sent and received
    for (int i = 0; i < 100; i++)
    {
        pub.publish(vel);
        c.update(time, duration);
        ros::spinOnce();
    }

    ASSERT_NEAR(handle_1.getCommand(), 0.0, 0.0001);
    ASSERT_NEAR(handle_2.getCommand(), 0.0, 0.0001);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "multi_joint_forward_controler_test");
    return RUN_ALL_TESTS();
}
