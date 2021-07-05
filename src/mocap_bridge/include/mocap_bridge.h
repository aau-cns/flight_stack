// Copyright (C) 2021 Christian Brommer and Alessandro Fornasier,
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the author at <christian.brommer@ieee.org>
// and <alessandro.fornasier@ieee.org>
//
// All rights reserved.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#ifndef MOCAPBRIDGE_H
#define MOCAPBRIDGE_H

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mocap_bridge/ExtCoreState.h>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <chrono>
#include <thread>

class MocapBridge {

public:

  /**
   * @brief Mocap Bridge constructor
   * @param Ros NodeHandle
   * @param subscriber topic
   * @param publisher topic
   */
  MocapBridge(ros::NodeHandle &nh, std::string &subscriber_topic, std::string &publisher_topic);

private:

  /**
   * @brief PoseStamped callback
   * @param const pointer to message
   */
  void callback_posestamped(const geometry_msgs::PoseStamped::ConstPtr& msg);

  /**
   * @brief Odometry callback
   * @param const pointer to message
   */
  void callback_odometry(const nav_msgs::Odometry::ConstPtr& msg);

  /// Nodehandler
  ros::NodeHandle nh_;

  /// Topics
  std::string subscriber_topic_, publisher_topic_;

  /// Publishers
  ros::Publisher pub_ecs_;

  /// Subscriber
  ros::Subscriber sub_posestamped_;
  ros::Subscriber sub_odometry_;

};

#endif // MOCAPBRIDGE_H
