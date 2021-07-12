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

MocapBridge::MocapBridge(ros::NodeHandle &nh, std::string &subscriber_topic, std::string &publisher_topic) :
  nh_(nh), subscriber_topic_(subscriber_topic), publisher_topic_(publisher_topic) {

  // Publishers
//  pub_ecs_ = nh.advertise<mocap_bridge::ExtCoreState>(publisher_topic_, 1);
//  ROS_INFO("Publishing: %s", pub_ecs_.getTopic().c_str());
  pub_ecsl_ = nh.advertise<mocap_bridge::ExtCoreStateLite>(publisher_topic_, 1);
  ROS_INFO("Publishing: %s", pub_ecsl_.getTopic().c_str());

  // Subscribers
//  sub_posestamped_ = nh.subscribe(subscriber_topic_, 1, &MocapBridge::callback_posestamped, this);
//  ROS_INFO("Subscribing: %s", sub_posestamped_.getTopic().c_str());
  sub_odometry_ = nh.subscribe(subscriber_topic_, 5, &MocapBridge::callback_odometry, this);
  ROS_INFO("Subscribing: %s", sub_odometry_.getTopic().c_str());

}

void MocapBridge::callback_posestamped(const geometry_msgs::PoseStamped::ConstPtr& msg) {

  // Define External Core State
  mocap_bridge::ExtCoreState ecs;

  // Assign mocap estimate to ecs
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

  // Publish
//  pub_ecs_.publish(ecs);

  // Define External Core State Lite
  mocap_bridge::ExtCoreState ecsl;

  // Assign mocap estimate to ecsl
  ecsl.header.stamp = msg->header.stamp;
  ecsl.header.seq = msg->header.seq;
  ecsl.header.frame_id = msg->header.frame_id;
  ecsl.p_wi.x = msg->pose.position.x;
  ecsl.p_wi.y = msg->pose.position.y;
  ecsl.p_wi.z = msg->pose.position.z;
  ecsl.q_wi.x = msg->pose.orientation.x;
  ecsl.q_wi.y = msg->pose.orientation.y;
  ecsl.q_wi.z = msg->pose.orientation.z;
  ecsl.q_wi.w = msg->pose.orientation.w;
  ecsl.QUATERNION_TYPE = 1; //Hamiltonian
  ecsl.FRAME_TYPE = 1; //ENU

  // Publish
  pub_ecsl_.publish(ecsl);
}

void MocapBridge::callback_odometry(const nav_msgs::Odometry::ConstPtr& msg) {

  // Define External Core State
  mocap_bridge::ExtCoreState ecs;

  // Assign mocap estimate to ecs
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

  // Publish
//  pub_ecs_.publish(ecs);

  // Define External Core State Lite
  mocap_bridge::ExtCoreStateLite ecsl;

  // Assign mocap estimate to ecs
  ecsl.header.stamp = msg->header.stamp;
  ecsl.header.seq = msg->header.seq;
  ecsl.header.frame_id = msg->header.frame_id;
  ecsl.p_wi.x = msg->pose.pose.position.x;
  ecsl.p_wi.y = msg->pose.pose.position.y;
  ecsl.p_wi.z = msg->pose.pose.position.z;
  ecsl.q_wi.x = msg->pose.pose.orientation.x;
  ecsl.q_wi.y = msg->pose.pose.orientation.y;
  ecsl.q_wi.z = msg->pose.pose.orientation.z;
  ecsl.q_wi.w = msg->pose.pose.orientation.w;
  ecsl.v_wi.x = msg->twist.twist.linear.x;
  ecsl.v_wi.y = msg->twist.twist.linear.y;
  ecsl.v_wi.z = msg->twist.twist.linear.z;
  ecsl.QUATERNION_TYPE = 1; //Hamiltonian
  ecsl.FRAME_TYPE = 1; //ENU

  // Publish
  pub_ecsl_.publish(ecsl);

}
