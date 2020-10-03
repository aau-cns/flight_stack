/******************************************************************************
* FILENAME:     main.cpp
* PURPOSE:      main
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     22.02.2018
*
*  Copyright (C) 2018
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include "evb1000_driver/TREK1000.hpp"
#include "evb1000_driver/EVK1000.hpp"

/**
 * The main function of the
 *
 * @param[in] argc argument count
 * @param[in] argv char* to argument list
 *
 */
int main(int argc, char **argv)
{
  // TODO: find better solution
#ifdef TREK1000_NODE
  std::string name("TREK1000_node");
#endif  // TREK1000_NODE
#ifdef EVK1000_NODE
  std::string name("EVK1000_node");
#endif  // EVK1000_NODE

  ROS_INFO("%s started", name.c_str());
  ros::init(argc, argv, name);
  ros::NodeHandle pnh("~");

#ifdef TREK1000_NODE
  evb1000_driver::TREK1000 nodeType(pnh);
  nodeType.run();
#endif  // TREK1000_NODE


#ifdef EVK1000_NODE
  evb1000_driver::EVK1000 nodeType(pnh);
  nodeType.run();
#endif  // EVK1000_NODE

  ROS_INFO("terminated");
}
