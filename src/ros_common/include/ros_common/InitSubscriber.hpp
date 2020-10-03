/**
 * @licence MIT
 *
 * @file  InitSubscriber.hpp
 *
 * @brief  subscribs a topic and wait for a single message.
 *         refactore init_subscriber.h
 *
 * @author  Roland Jung
 *
 * @date  14.12.2016
 */
#ifndef ROS_COMMON_INITSUBSCRIBER_HPP
#define ROS_COMMON_INITSUBSCRIBER_HPP

#include <functional>
#include <string>
#include <ros/ros.h>
#include <atomic>

namespace ros_common
{

  /**
     * Template class to wait for a specific ROS topic to be received.
     *
     */

  template<typename T>
  class InitSubscriber
  {

    private:
      std::string        mTopic;
      ros::NodeHandle    mNh;
      ros::Subscriber    mSub;
      std::atomic<bool>  mbReceived;
    public:
      T data;
      boost::shared_ptr< T const> ptrData;

      InitSubscriber(ros::NodeHandle &nh) :  mNh(nh)
      {
        mbReceived = false;
      }

      InitSubscriber(ros::NodeHandle &nh, std::string const &topic) : mNh(nh), mTopic(topic)
      {
        mbReceived = false;
        subscribe();
      }

      void callback(boost::shared_ptr< T const> msg)
      {
        ptrData = msg;
        data = *msg;
        mSub.shutdown();
        mbReceived = true;
      }

      void subscribe(std::string const &topic = "")
      {
        if(!topic.empty())
        {
          mTopic = topic;
        }
        mbReceived = false;
        mSub      = mNh.subscribe<T>(mTopic, 1, boost::bind(&InitSubscriber<T>::callback, this, _1));
        ROS_INFO("InitSubscriber: subscribed topic %s", mTopic.c_str());
      }

      void waitForMessage(double rate_Hz = 10.0)
      {
        ROS_INFO("Waiting for message: %s", mTopic.c_str());

        ros::Rate    rate(rate_Hz);
        while (!mbReceived)
        {
          usleep(100000);
          ros::spinOnce();
          rate.sleep();
        }
        ROS_INFO(" ... received message");
      }

  };

} // namespace ros_common

#endif // ROS_COMMON_INITSUBSCRIBER_HPP
