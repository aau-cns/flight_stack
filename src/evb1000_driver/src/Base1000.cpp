/******************************************************************************
* FILENAME:     Base1000.cpp
* PURPOSE:      Base1000
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     22.02.2018
*
*  Copyright (C) 2018
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include "evb1000_driver/Base1000.hpp"
#include <iomanip>
#include <limits>

evb1000_driver::Base1000::Base1000(ros::NodeHandle &_nh) : INode(_nh)
{
  mbSpinningProcess = true; // process will be called at spinning rate!
  mbRecording = false;
}

bool evb1000_driver::Base1000::update_()
{
  ROS_INFO("Reconfigure Request:%s %s",
           mDynConfig.filename.c_str(),
           mDynConfig.record?"True":"False");

  if(mbRecording && !mDynConfig.record)
  {
    // close the file!
    file.close();
    mbRecording = false;
  }
  if(!mbRecording && mDynConfig.record)
  {
    // open a new file!
    file.open(mDynConfig.filename);

    ROS_ERROR_STREAM_COND(!file.is_open(), "Can't open file " << mDynConfig.filename);
    ROS_ASSERT_MSG(file.is_open(), "Can't open file!");
    mbRecording = true;

    file << std::setprecision (std::numeric_limits<double>::max_digits10);
    print_header(file);
  }

  return true;
}

void evb1000_driver::Base1000::load_config()
{
  mNh.param<std::string>("serial_device", serial_device, "/dev/ttyACM0");
  mNh.param<bool>("bVerbose", mbVerbose, false);
}

void evb1000_driver::Base1000::init()
{
  device.open(serial_device.c_str(), std::ifstream::in);
  ROS_ERROR_STREAM_COND(!device.is_open(), "Can't open device " << serial_device);
  ROS_ASSERT_MSG(device.is_open(), "Can't open device! Correct access right?");
}

void evb1000_driver::Base1000::process()
{
  std::string rawLine;
  std::getline(device, rawLine);  // blocking

  bool success = parse_data(rawLine);

  if(success)
  {
    publish_data();

    if(mbRecording)
    {
      dump_to_file(file);
    }
  }
}

unsigned int evb1000_driver::hexToUint(const std::string &inp)
{
  // convert hex to unsigned integer
  // ex: "00ff" => 255
  unsigned int x;
  std::stringstream ss;
  ss << std::hex << inp;
  ss >> x;
  return x;
}

float evb1000_driver::hexToMeter(const std::string &inp)
{
  unsigned int mm = hexToUint(inp);
  return (float)mm/1000.0;
}
