#include <geometry_transformer_util.h>
#include <gtest/gtest.h>


TEST(transform_wrench_test, test_frame_id)
{
    geometry_msgs::WrenchStamped wrench_transformed;
    geometry_msgs::WrenchStamped wrench_in;
    tf::StampedTransform transform;

    transform.frame_id_ = "abc";
    wrench_transformed = transform_wrench(transform, wrench_in);
    EXPECT_TRUE("abc" == wrench_transformed.header.frame_id);

    transform.frame_id_ = "def";
    wrench_transformed = transform_wrench(transform, wrench_in);
    EXPECT_TRUE("def" == wrench_transformed.header.frame_id);
}


TEST(transform_wrench_test, test_stamp)
{
    geometry_msgs::WrenchStamped wrench_transformed;
    geometry_msgs::WrenchStamped wrench_in;
    tf::StampedTransform transform;

    wrench_in.header.stamp = ros::Time(0.0);
    wrench_transformed = transform_wrench(transform, wrench_in);
    EXPECT_EQ(ros::Time(0.0), wrench_transformed.header.stamp);

    wrench_in.header.stamp = ros::Time(0.5);
    wrench_transformed = transform_wrench(transform, wrench_in);
    EXPECT_EQ(ros::Time(0.5), wrench_transformed.header.stamp);
}


TEST(transform_wrench_test, test_identity_transform)
{
    geometry_msgs::WrenchStamped wrench_transformed;
    geometry_msgs::WrenchStamped wrench_in;
    tf::StampedTransform transform;

    transform.setIdentity();

    wrench_in.wrench.force.x = 0.0;
    wrench_in.wrench.force.y = 0.0;
    wrench_in.wrench.force.z = 0.0;
    wrench_in.wrench.torque.x = 0.0;
    wrench_in.wrench.torque.y = 0.0;
    wrench_in.wrench.torque.z = 0.0;
    wrench_transformed = transform_wrench(transform, wrench_in);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.force.x, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.force.y, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.force.z, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.torque.x, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.torque.y, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.torque.z, 0.0001);

    wrench_in.wrench.force.x = 1.0;
    wrench_in.wrench.force.y = 2.0;
    wrench_in.wrench.force.z = 3.0;
    wrench_in.wrench.torque.x = 4.0;
    wrench_in.wrench.torque.y = 5.0;
    wrench_in.wrench.torque.z = 6.0;
    wrench_transformed = transform_wrench(transform, wrench_in);
    EXPECT_NEAR(1.0, wrench_transformed.wrench.force.x, 0.0001);
    EXPECT_NEAR(2.0, wrench_transformed.wrench.force.y, 0.0001);
    EXPECT_NEAR(3.0, wrench_transformed.wrench.force.z, 0.0001);
    EXPECT_NEAR(4.0, wrench_transformed.wrench.torque.x, 0.0001);
    EXPECT_NEAR(5.0, wrench_transformed.wrench.torque.y, 0.0001);
    EXPECT_NEAR(6.0, wrench_transformed.wrench.torque.z, 0.0001);
}


TEST(transform_wrench_test, test_translation_transform)
{
    geometry_msgs::WrenchStamped wrench_transformed;
    geometry_msgs::WrenchStamped wrench_in;
    tf::StampedTransform transform;

    transform.setIdentity();
    transform.setOrigin(tf::Vector3(1.0, 2.0, 3.0));

    wrench_in.wrench.force.x = 0.0;
    wrench_in.wrench.force.y = 0.0;
    wrench_in.wrench.force.z = 0.0;
    wrench_in.wrench.torque.x = 0.0;
    wrench_in.wrench.torque.y = 0.0;
    wrench_in.wrench.torque.z = 0.0;
    wrench_transformed = transform_wrench(transform, wrench_in);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.force.x, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.force.y, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.force.z, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.torque.x, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.torque.y, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.torque.z, 0.0001);

    wrench_in.wrench.force.x = 1.0;
    wrench_in.wrench.force.y = 2.0;
    wrench_in.wrench.force.z = 3.0;
    wrench_in.wrench.torque.x = 4.0;
    wrench_in.wrench.torque.y = 5.0;
    wrench_in.wrench.torque.z = 6.0;
    wrench_transformed = transform_wrench(transform, wrench_in);
    EXPECT_NEAR(1.0, wrench_transformed.wrench.force.x, 0.0001);
    EXPECT_NEAR(2.0, wrench_transformed.wrench.force.y, 0.0001);
    EXPECT_NEAR(3.0, wrench_transformed.wrench.force.z, 0.0001);
    EXPECT_NEAR(4.0, wrench_transformed.wrench.torque.x, 0.0001);
    EXPECT_NEAR(5.0, wrench_transformed.wrench.torque.y, 0.0001);
    EXPECT_NEAR(6.0, wrench_transformed.wrench.torque.z, 0.0001);

    transform.setOrigin(tf::Vector3(1.0, 0.0, 0.0));

    wrench_in.wrench.force.x = 0.0;
    wrench_in.wrench.force.y = 1.0;
    wrench_in.wrench.force.z = 0.0;
    wrench_in.wrench.torque.x = 0.0;
    wrench_in.wrench.torque.y = 0.0;
    wrench_in.wrench.torque.z = 0.0;
    wrench_transformed = transform_wrench(transform, wrench_in);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.force.x, 0.0001);
    EXPECT_NEAR(1.0, wrench_transformed.wrench.force.y, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.force.z, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.torque.x, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.torque.y, 0.0001);
    EXPECT_NEAR(1.0, wrench_transformed.wrench.torque.z, 0.0001);
}


TEST(transform_wrench_test, test_orientation_roll_transform)
{
    geometry_msgs::WrenchStamped wrench_transformed;
    geometry_msgs::WrenchStamped wrench_in;
    tf::StampedTransform transform;

    transform.setIdentity();

    transform.setRotation(tf::createQuaternionFromRPY(M_PI_2, 0.0, 0.0));
    wrench_in.wrench.force.x = 1.0;
    wrench_in.wrench.force.y = 0.0;
    wrench_in.wrench.force.z = 0.0;
    wrench_in.wrench.torque.x = 1.0;
    wrench_in.wrench.torque.y = 0.0;
    wrench_in.wrench.torque.z = 0.0;
    wrench_transformed = transform_wrench(transform, wrench_in);
    EXPECT_NEAR(1.0, wrench_transformed.wrench.force.x, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.force.y, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.force.z, 0.0001);
    EXPECT_NEAR(1.0, wrench_transformed.wrench.torque.x, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.torque.y, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.torque.z, 0.0001);
}


TEST(transform_wrench_test, test_orientation_pitch_transform)
{
    geometry_msgs::WrenchStamped wrench_transformed;
    geometry_msgs::WrenchStamped wrench_in;
    tf::StampedTransform transform;

    transform.setIdentity();

    transform.setRotation(tf::createQuaternionFromRPY(0.0, M_PI_2, 0.0));
    wrench_in.wrench.force.x = 1.0;
    wrench_in.wrench.force.y = 0.0;
    wrench_in.wrench.force.z = 0.0;
    wrench_in.wrench.torque.x = 1.0;
    wrench_in.wrench.torque.y = 0.0;
    wrench_in.wrench.torque.z = 0.0;
    wrench_transformed = transform_wrench(transform, wrench_in);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.force.x, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.force.y, 0.0001);
    EXPECT_NEAR(-1.0, wrench_transformed.wrench.force.z, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.torque.x, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.torque.y, 0.0001);
    EXPECT_NEAR(-1.0, wrench_transformed.wrench.torque.z, 0.0001);
}


TEST(transform_wrench_test, test_orientation_yaw_transform)
{
    geometry_msgs::WrenchStamped wrench_transformed;
    geometry_msgs::WrenchStamped wrench_in;
    tf::StampedTransform transform;

    transform.setIdentity();

    transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, M_PI_2));
    wrench_in.wrench.force.x = 1.0;
    wrench_in.wrench.force.y = 0.0;
    wrench_in.wrench.force.z = 0.0;
    wrench_in.wrench.torque.x = 1.0;
    wrench_in.wrench.torque.y = 0.0;
    wrench_in.wrench.torque.z = 0.0;
    wrench_transformed = transform_wrench(transform, wrench_in);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.force.x, 0.0001);
    EXPECT_NEAR(1.0, wrench_transformed.wrench.force.y, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.force.z, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.torque.x, 0.0001);
    EXPECT_NEAR(1.0, wrench_transformed.wrench.torque.y, 0.0001);
    EXPECT_NEAR(0.0, wrench_transformed.wrench.torque.z, 0.0001);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
