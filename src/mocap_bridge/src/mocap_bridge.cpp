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

#include "mocap_bridge.h"
#include "utils/colors.h"

MocapBridge::MocapBridge(ros::NodeHandle &nh, std::string &subscriber_topic, std::string &subscriber_type, std::string &publisher_topic, std::string &publisher_type) :
  nh_(nh), subscriber_topic_(subscriber_topic), subscriber_type_(subscriber_type), publisher_topic_(publisher_topic) {

  // Define publishers
  if ((publisher_type.compare("full") >> 1) == 0) {
    publisher_type_ = publisherType::FULL;
    pub_ = nh.advertise<mocap_bridge::ExtCoreState>(publisher_topic_, 1);
  } else if ((publisher_type.compare("lite") >> 1) == 0) {
    publisher_type_ = publisherType::LITE;
    pub_ = nh.advertise<mocap_bridge::ExtCoreStateLite>(publisher_topic_, 1);
  } else {
    publisher_type_ = publisherType::FULL;
    ROS_WARN("Invalid publisher type, setting to [full] by default");
    pub_ = nh.advertise<mocap_bridge::ExtCoreState>(publisher_topic_, 1);
  }
  ROS_INFO("Publishing: %s", pub_.getTopic().c_str());

  // Define subscribers
  if ((subscriber_type_.compare("pose") >> 1) == 0) {
    sub_ = nh.subscribe(subscriber_topic_, 1, &MocapBridge::callback_posestamped, this);
  } else if ((subscriber_type_.compare("odometry") >> 1) == 0) {
    sub_ = nh.subscribe(subscriber_topic_, 5, &MocapBridge::callback_odometry, this);
  } else {
    ROS_WARN("Invalid subscriber type, setting to [pose] by default");
    sub_ = nh.subscribe(subscriber_topic_, 1, &MocapBridge::callback_posestamped, this);
  }
  ROS_INFO("Subscribing: %s", sub_.getTopic().c_str());

}

void MocapBridge::callback_posestamped(const geometry_msgs::PoseStamped::ConstPtr& msg) {

  if (publisher_type_ == publisherType::FULL) {
    mocap_bridge::ExtCoreState ecs;
    ecs.header.stamp = msg->header.stamp;
    ecs.header.seq = msg->header.seq;
    ecs.header.frame_id = msg->header.frame_id;
    ecs.p_wi.x = msg->pose.position.x;
    ecs.p_wi.y = msg->pose.position.y;
    ecs.p_wi.z = msg->pose.position.z;
    ecs.q_wi.x = msg->pose.orientation.x;
    ecs.q_wi.y = msg->pose.orientation.y;
    ecs.q_wi.z = msg->pose.orientation.z;
    ecs.q_wi.w = msg->pose.orientation.w;
    ecs.QUATERNION_TYPE = 1; //Hamiltonian
    ecs.FRAME_TYPE = 1; //ENU
    pub_.publish(ecs);
  } else if (publisher_type_ == publisherType::FULL) {
    mocap_bridge::ExtCoreStateLite ecs;
    ecs.header.stamp = msg->header.stamp;
    ecs.header.seq = msg->header.seq;
    ecs.header.frame_id = msg->header.frame_id;
    ecs.p_wi.x = msg->pose.position.x;
    ecs.p_wi.y = msg->pose.position.y;
    ecs.p_wi.z = msg->pose.position.z;
    ecs.q_wi.x = msg->pose.orientation.x;
    ecs.q_wi.y = msg->pose.orientation.y;
    ecs.q_wi.z = msg->pose.orientation.z;
    ecs.q_wi.w = msg->pose.orientation.w;
    ecs.QUATERNION_TYPE = 1; //Hamiltonian
    ecs.FRAME_TYPE = 1; //ENU
    pub_.publish(ecs);
  } else {
    ROS_ERROR("Wrong publisher type");
  }
}

void MocapBridge::callback_odometry(const nav_msgs::Odometry::ConstPtr& msg) {

  if (publisher_type_ == publisherType::FULL) {
    mocap_bridge::ExtCoreState ecs;
    ecs.header.stamp = msg->header.stamp;
    ecs.header.seq = msg->header.seq;
    ecs.header.frame_id = msg->header.frame_id;
    ecs.p_wi.x = msg->pose.pose.position.x;
    ecs.p_wi.y = msg->pose.pose.position.y;
    ecs.p_wi.z = msg->pose.pose.position.z;
    ecs.q_wi.x = msg->pose.pose.orientation.x;
    ecs.q_wi.y = msg->pose.pose.orientation.y;
    ecs.q_wi.z = msg->pose.pose.orientation.z;
    ecs.q_wi.w = msg->pose.pose.orientation.w;
    ecs.v_wi.x = msg->twist.twist.linear.x;
    ecs.v_wi.y = msg->twist.twist.linear.y;
    ecs.v_wi.z = msg->twist.twist.linear.z;
    ecs.QUATERNION_TYPE = 1; //Hamiltonian
    ecs.FRAME_TYPE = 1; //ENU
    pub_.publish(ecs);
  } else if (publisher_type_ == publisherType::FULL) {
    mocap_bridge::ExtCoreStateLite ecs;
    ecs.header.stamp = msg->header.stamp;
    ecs.header.seq = msg->header.seq;
    ecs.header.frame_id = msg->header.frame_id;
    ecs.p_wi.x = msg->pose.pose.position.x;
    ecs.p_wi.y = msg->pose.pose.position.y;
    ecs.p_wi.z = msg->pose.pose.position.z;
    ecs.q_wi.x = msg->pose.pose.orientation.x;
    ecs.q_wi.y = msg->pose.pose.orientation.y;
    ecs.q_wi.z = msg->pose.pose.orientation.z;
    ecs.q_wi.w = msg->pose.pose.orientation.w;
    ecs.v_wi.x = msg->twist.twist.linear.x;
    ecs.v_wi.y = msg->twist.twist.linear.y;
    ecs.v_wi.z = msg->twist.twist.linear.z;
    ecs.QUATERNION_TYPE = 1; //Hamiltonian
    ecs.FRAME_TYPE = 1; //ENU
    pub_.publish(ecs);
  } else {
    ROS_ERROR("Wrong publisher type");
  }

}
