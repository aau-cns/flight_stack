// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@aau.at>

#include "ros_node_killer.hpp"

namespace ros_killer
{
RosNodeKiller::RosNodeKiller(ros::NodeHandle& nh) : nh_(nh)
{
  sub_response = nh_.subscribe("/autonomy/response", 1, &RosNodeKiller::cbMissionResponse, this);

  nh.param<std::string>("node_name", node_name_, node_name_);
  nh.param<double>("wait_time_s", wait_time_after_to_s_, wait_time_after_to_s_);
}

void RosNodeKiller::cbMissionResponse(const mission_sequencer::MissionResponse::ConstPtr& msg)
{
  // check if msg is of type Takeoff
  if (msg->request.request == mission_sequencer::MissionRequest::TAKEOFF)
  {
    // check if takeoff has been completed
    if (msg->completed)
    {
      // start timer
      // for now sleep for X seconds
      ros::Duration(wait_time_after_to_s_).sleep();

      // kill
      killRosNode();
    }
  }
}

void RosNodeKiller::killRosNode()
{
  std::string cmd = "bash -c 'source ~/.bashrc; rosnode kill " + node_name_ + "'";
  ROS_INFO_STREAM("[RosNodeKiller] executing: " << cmd);
  if(!std::system(cmd.c_str()))
    ROS_ERROR_STREAM("[RosNodeKiller] Failure in executing system call.");
}

}  // namespace ros_killer

// new stuff
