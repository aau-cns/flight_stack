// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
// You can contact the author at <martin.scheiber@aau.at>

#ifndef ROS_NODE_KILLER_HPP_
#define ROS_NODE_KILLER_HPP_

#include <mission_sequencer/MissionResponse.h>
#include <ros/ros.h>

#include <Eigen/Eigen>

namespace ros_killer
{
class RosNodeKiller
{
public:
  ///
  /// \brief UwbInitWrapper default constructor for UwbInitWrapper
  /// \param nh ros nodehandle
  /// \param params node parameters set by the launchfile
  ///
  RosNodeKiller(ros::NodeHandle& nh);

  void cbTimerInit(const ros::TimerEvent&);

private:
  void cbMissionResponse(const mission_sequencer::MissionResponse::ConstPtr& msg);

  void killRosNode();

  // Ros node handler
  ros::NodeHandle nh_;  //!< ROS nodehandle given through constructor

  // Subscribers
  ros::Subscriber sub_response;  //!< ROS subscriber for poses of IMU (or body) in global frame

  // Publishers
  ros::Publisher pub_anchor;  //!< ROS publisher for anchor position and biases
  ros::Publisher pub_wplist;  //!< ROS publisher for wp list

  // Params
  double wait_time_after_to_s_{ 2 };
  std::string node_name_{ "/camera_nodelet" };

  // timer variables
  ros::Timer init_check_timer_;  //!< timer used to check and perform initialization
};

}  // namespace ros_killer

#endif  // ROS_NODE_KILLER_HPP_
