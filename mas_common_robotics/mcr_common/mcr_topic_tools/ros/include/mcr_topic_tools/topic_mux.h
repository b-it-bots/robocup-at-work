///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2009, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Changes:
// 06-2015: Santosh: Converted functionality into a nodelet and to use
//          publisher/subcriber to select input topic.
/////////////////////////////////////////////////////////////////////////////

#ifndef MCR_TOPIC_TOOLS_TOPIC_MUX_H_
#define MCR_TOPIC_TOOLS_TOPIC_MUX_H_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>

#include <topic_tools/shape_shifter.h>
#include <topic_tools/parse.h>


/**
 * This nodelet subscribes to a list of input topics and publishes the selected input topic to
 * the output topic; i.e. it multiplexes a set of input topics
 * The original implementation is here: https://github.com/ros/ros_comm/blob/hydro-devel/tools/topic_tools/src/mux.cpp
 *
 * Subscribes:
 *      various input topics specified as arguments to the nodelet
 *      ~/select : topic on which to specify which input topic to pipe to the output
 *      ~/event_in: switches to next topic if e_trigger is published on this topic
 *
 * Publishes:
 *      output topic specified as an argument to the nodelet
 *      ~/event_out: e_done if switch is successful, e_failed if topic is not found
 *
 */
namespace mcr_topic_tools
{

class TopicMux : public nodelet::Nodelet
{
public:
    TopicMux();
    virtual ~TopicMux();


private:
    virtual void onInit();
    /**
     * Callback for output publisher. This gets called when someone subscribes to the output topic
     * If an input topic has been selected and we're in lazy mode, we subscribe to the input topic here
     */
    void connectionCallback();
    /**
     * Callback for subscription to ~/select
     * The name of the input topic to select is to be published on this topic by another node
     */
    void selectTopicCallback(const std_msgs::String::Ptr &topic_name);
    /**
     * The callback for the selected input topic. If there are subscribers for the output topic,
     * this message is directly published to the output topic.
     *
     * @param msg
     *      message published on the input topic
     * @param s
     *      pointer to message type of selected subscription
     *      This is used to make sure the input message is for the selected subscription in the case
     *      that we're subscribed to multiple input topics at once (in non-lazy mode)
     */
    void inputTopicCallback(const topic_tools::ShapeShifter::ConstPtr &msg, topic_tools::ShapeShifter::Ptr s);
    /**
     * Callback for event_in topic
     * If e_trigger is published to event_in, the next topic in subscriptions_ is chosen as the 
     * selected_subscription_ (loops back to the beginning of the list)
     */
    void eventInCallback(const std_msgs::String::Ptr &msg);



private:
    /**
     * describes an input topic subscription
     */
    struct SubscriberInfo
    {
        std::string topic;
        boost::shared_ptr<ros::Subscriber> subscriber;
        topic_tools::ShapeShifter::Ptr message;
    };

    /**
     * List of subscriptions
     */
    std::vector<SubscriberInfo> subscriptions_;
    /**
     * selected subscription
     */
    std::vector<SubscriberInfo>::iterator selected_subscription_;

    std::string output_topic_;
    /**
     * This parameter defines whether or not we subscribe to all input topics or only the selected
     * input topic
     * True: subscribe only to selected input topic
     * False: subscribe to all input topics
     */
    bool is_lazy_;
    /**
     * Flag indicating whether the output topic has been advertised or not
     */
    bool is_advertised_;

    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    ros::Subscriber sub_select_topic_;

    ros::Subscriber sub_event_in_;
    ros::Publisher pub_event_out_;
};
PLUGINLIB_DECLARE_CLASS(mcr_topic_tools, TopicMux, mcr_topic_tools::TopicMux, nodelet::Nodelet);
} /* namespace mcr_topic_tools */

#endif  // MCR_TOPIC_TOOLS_TOPIC_MUX_H_
