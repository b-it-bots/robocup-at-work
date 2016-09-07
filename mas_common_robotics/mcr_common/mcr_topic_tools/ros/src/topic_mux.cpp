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

#include <mcr_topic_tools/topic_mux.h>
#include <string>
#include <vector>

using  mcr_topic_tools::TopicMux;

TopicMux::TopicMux() : is_advertised_(false), selected_subscription_(subscriptions_.end())
{
}

TopicMux::~TopicMux()
{
    for (std::vector<SubscriberInfo>::iterator it = subscriptions_.begin();
            it != subscriptions_.end();
            ++it)
    {
        if (it->subscriber)
        {
            it->subscriber->shutdown();
        }
    }

    subscriptions_.clear();
}

void TopicMux::onInit()
{
    std::vector<std::string> args = getMyArgv();

    private_nh_ = getPrivateNodeHandle();
    nh_ = getNodeHandle();

    if (args.size() < 3)
    {
        NODELET_ERROR("Not enough arguments provided");
        return;
    }

    output_topic_ = args[0];

    private_nh_.getParam("lazy", is_lazy_);

    // get list of input topics
    for (size_t i = 1; i < args.size(); i++)
    {
        SubscriberInfo subscription;
        subscription.topic = ros::names::resolve(args[i]);
        subscription.message.reset(new topic_tools::ShapeShifter);
        subscription.subscriber.reset(new ros::Subscriber(
                                       nh_.subscribe<topic_tools::ShapeShifter>(subscription.topic, 10,
                                        boost::bind(&TopicMux::inputTopicCallback, this, _1, subscription.message))));
        subscriptions_.push_back(subscription);
    }

    // by default select the first input topic
    selected_subscription_ = subscriptions_.begin();

    sub_select_topic_ = private_nh_.subscribe("select", 1, &TopicMux::selectTopicCallback, this);
    sub_event_in_ = private_nh_.subscribe("event_in", 1, &TopicMux::eventInCallback, this);
    pub_event_out_ = private_nh_.advertise<std_msgs::String>("event_out", 1);
}

void TopicMux::connectionCallback()
{
    // when someone subscribes to the output topic, check that we're publishing the selected input topic
    if (is_lazy_ && selected_subscription_ != subscriptions_.end() && !selected_subscription_->subscriber)
    {
        NODELET_INFO("Subscribing to %s", selected_subscription_->topic.c_str());
        selected_subscription_->subscriber.reset(new ros::Subscriber(
                    nh_.subscribe<topic_tools::ShapeShifter>(selected_subscription_->topic, 10,
                            boost::bind(&TopicMux::inputTopicCallback, this, _1, selected_subscription_->message))));
    }
}


void TopicMux::selectTopicCallback(const std_msgs::String::Ptr &topic_name)
{
    // if the selected topic is not the currently selected one, unsubscribe from it (if in lazy mode)
    if (selected_subscription_ != subscriptions_.end() && is_lazy_ && selected_subscription_->subscriber
            && selected_subscription_->topic != topic_name->data)
    {
        NODELET_INFO("Unsubscribing from %s", selected_subscription_->topic.c_str());
        selected_subscription_->subscriber->shutdown();
        selected_subscription_->subscriber.reset();
    }

    bool found_topic = false;
    // if we don't want to subscribe to anything
    if (topic_name->data == "__none")
    {
        NODELET_INFO("Subscribed to no topic");
        selected_subscription_ = subscriptions_.end();
        found_topic = true;
    }
    else
    {
        std::vector<SubscriberInfo>::iterator it;

        for (it = subscriptions_.begin(); it != subscriptions_.end(); ++it)
        {
            if (ros::names::resolve(it->topic) == ros::names::resolve(topic_name->data))
            {
                selected_subscription_ = it;
                NODELET_INFO("Selected topic: %s", selected_subscription_->topic.c_str());

                if (!selected_subscription_->subscriber &&
                    (!is_advertised_ || (is_advertised_ && publisher_.getNumSubscribers())))
                {
                    selected_subscription_->subscriber.reset(
                        new ros::Subscriber(nh_.subscribe<topic_tools::ShapeShifter>(selected_subscription_->topic, 10,
                                                                boost::bind(&TopicMux::inputTopicCallback, this, _1,
                                                                selected_subscription_->message))));
                }
                found_topic = true;
            }
        }
    }
    std_msgs::String event_out;
    if (found_topic)
    {
        event_out.data = "e_done";
    }
    else
    {
        event_out.data = "e_failed";
    }
    pub_event_out_.publish(event_out);
}

void TopicMux::inputTopicCallback(const topic_tools::ShapeShifter::ConstPtr &msg, topic_tools::ShapeShifter::Ptr s)
{
    if (!is_advertised_)
    {
        NODELET_INFO("Advertising topic: %s", output_topic_.c_str());
        publisher_ = msg->advertise(nh_, output_topic_, 10, false, boost::bind(&TopicMux::connectionCallback, this));
        is_advertised_ = true;

        if (is_lazy_)
        {
            for (std::vector<SubscriberInfo>::iterator it = subscriptions_.begin(); it != subscriptions_.end(); ++it)
            {
                if (it != selected_subscription_)
                {
                    NODELET_INFO("Unsubscribing: from %s", it->topic.c_str());

                    if (it->subscriber)
                    {
                        it->subscriber->shutdown();
                    }

                    it->subscriber.reset();
                }
            }
        }
    }

    // make sure we've got a message for the selected input topic and not one of the other input topics
    if (s != selected_subscription_->message)
    {
        return;
    }

    if (is_lazy_ && !publisher_.getNumSubscribers() && selected_subscription_ != subscriptions_.end())
    {
        NODELET_INFO("No subscribers. Unsubscribing from %s", selected_subscription_->topic.c_str());
        selected_subscription_->subscriber->shutdown();
        selected_subscription_->subscriber.reset();
    }
    else
    {
        publisher_.publish(msg);
    }
}

void TopicMux::eventInCallback(const std_msgs::String::Ptr &msg)
{
    std_msgs::String::Ptr topic_name(new std_msgs::String);
    if (msg->data == "e_trigger")
    {
        std::vector<SubscriberInfo>::iterator it = selected_subscription_;
        // select next topic
        it++;
        // cycle back to front of list if we've reached the end
        if (it == subscriptions_.end())
        {
            topic_name->data = subscriptions_.begin()->topic;
        }
        else
        {
            topic_name->data = it->topic;
        }
        selectTopicCallback(topic_name);
    }
}
