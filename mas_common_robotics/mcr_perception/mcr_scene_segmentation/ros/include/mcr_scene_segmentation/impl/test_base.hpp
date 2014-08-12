#ifndef TEST_BASE_HPP_
#define TEST_BASE_HPP_

#include <iostream>

#include <boost/lexical_cast.hpp>
#include <pcl/common/time.h>
#include <pcl/filters/passthrough.h>
#include <ros/console.h>

#define MEASURE_RUNTIME(function, description) \
        { \
          double ts1 = pcl::getTime(); \
          (function); \
          double ts2 = pcl::getTime(); \
          std::cout << (description) << " took " << ts2 - ts1 << " seconds." << std::endl; \
        }

class TestBase
{

public:

  virtual ~TestBase() { }

  void setupPassThroughFilter(const char* min, const char* max)
  {
    float min_z = boost::lexical_cast<float>(min);
    float max_z = boost::lexical_cast<float>(max);
    setupPassThroughFilter(min_z, max_z);
  }

  void setupPassThroughFilter(float min_z, float max_z)
  {
    pass_through_.reset(new pcl::PassThrough<PointT>);
    pass_through_->setFilterFieldName("z");
    pass_through_->setFilterLimits(min_z, max_z);
    pass_through_->setKeepOrganized(true);
    ROS_INFO("PassThrough filter set to keep points in %.2f to %.2f range.", min_z, max_z);
  }

protected:

  virtual void process() = 0;

  inline void applyPassThroughFilter(PointCloud::Ptr& cloud)
  {
    if (pass_through_)
    {
      pass_through_->setInputCloud(cloud);
      pass_through_->filter(*cloud);
    }
  }

  std::unique_ptr<pcl::PassThrough<PointT>> pass_through_;

};

#endif

