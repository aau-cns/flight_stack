/******************************************************************************
* FILENAME:     EVK1000.cpp
* PURPOSE:      EVK1000
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     22.02.2018
*
*  Copyright (C) 2018
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include "evb1000_driver/EVK1000.hpp"
#include <boost/algorithm/string/regex.hpp>
#include <boost/regex.hpp>

const boost::regex pattern("(?<ALSB>ia[[:xdigit:]]{4}) (?<TLSB>t[[:xdigit:]]{4}) (?<range_cor>[[:xdigit:]]{8}) "
                           "(?<range_raw>[[:xdigit:]]{8}) (?<nranges>[[:xdigit:]]{4}) (?<TXdelay>[[:xdigit:]]{4}) (?<RXdelay>[[:xdigit:]]{4}) "
                           "(?<anchor_or_tag>a|t)", boost::regex::normal);

evb1000_driver::EVK1000::EVK1000(ros::NodeHandle &_nh) : Base1000(_nh)
{

}

void evb1000_driver::EVK1000::init_topics()
{
  // range distance between anchor and tag
  mPubRangeMeasurement = mNh.advertise<evb1000_driver::RangingMeasurement>("/rangeMesasurement", 1);
}

bool evb1000_driver::EVK1000::parse_data(const std::string &rawline)
{
  boost::smatch element;
  if(boost::regex_search(rawline, element, pattern))
  {
    evb1000_driver::RangingMeasurement msg;
    msg.header.seq = hexToUint(element["nranges"]);
    msg.header.stamp = ros::Time::now();
    if(element["anchor_or_tag"] == "t")
    {
      msg.header.frame_id = "tag";
    }
    else
    {
      msg.header.frame_id = "anchor";
    }

    msg.distance_cor = evb1000_driver::hexToMeter(element["range_cor"]);
    msg.distance_raw = evb1000_driver::hexToMeter(element["range_raw"]);

    if(mbVerbose)
    {
      ROS_INFO_THROTTLE(1, "EVK1000: %s, %d, %f m, %f m ", std::string(msg.header.frame_id).c_str(), msg.header.seq, msg.distance_cor,
                        msg.distance_raw);
    }
    mMsg = msg;
    return true;
  }
  return false;
}

void evb1000_driver::EVK1000::print_header(std::ofstream &file)
{
  file << "# v1.0 firmware: DecaRanging ARM Source Code v3.05" << std::endl;
  file << "# timestamp[sec] distance_cor[m] distance_rawm] type seq_num" << std::endl;
}

void evb1000_driver::EVK1000::dump_to_file(std::ofstream &file)
{
  file << mMsg.header.stamp.toSec() << " " << mMsg.distance_cor << " " << mMsg.distance_raw << " " << mMsg.header.frame_id << " " <<
       mMsg.header.seq << std::endl;
}

void evb1000_driver::EVK1000::publish_data()
{
  mPubRangeMeasurement.publish(mMsg);
}
