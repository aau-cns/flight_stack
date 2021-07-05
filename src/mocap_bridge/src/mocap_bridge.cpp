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
  pub_ecs_ = nh.advertise<mocap_bridge::ExtCoreState>(publisher_topic_, 1);
  ROS_INFO("Publishing: %s", pub_ecs_.getTopic().c_str());

  // Subscribers
  sub_posestamped_ = nh.subscribe(subscriber_topic_, 1, &MocapBridge::callback_posestamped, this);
  ROS_INFO("Subscribing: %s", sub_posestamped_.getTopic().c_str());

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
  pub_ecs_.publish(ecs);
}
