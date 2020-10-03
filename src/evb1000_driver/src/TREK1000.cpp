/******************************************************************************
* FILENAME:     TREK1000.cpp
* PURPOSE:      TREK1000
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     22.02.2018
*
*  Copyright (C) 2018
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include "evb1000_driver/TREK1000.hpp"
#include <boost/algorithm/string/regex.hpp>
#include <boost/regex.hpp>

// See page 13 of DecaRangeRTLS_ARM_Source_Code_Guide.pdf for the definition of the format.
// This Regex matches exactly with one message.
const boost::regex pattern_trek1000("(?<mid>mr|mc|ma) (?<mask>[[:xdigit:]]{2}) (?<range0>[[:xdigit:]]{8}) "
                                    "(?<range1>[[:xdigit:]]{8}) (?<range2>[[:xdigit:]]{8}) (?<range3>[[:xdigit:]]{8}) "
                                    "(?<nranges>[[:xdigit:]]{4}) (?<rseq>[[:xdigit:]]{2}) (?<debug>[[:xdigit:]]{8}) "
                                    "(?<anchor_or_tag>a|t)(?<tagID>\\d):(?<anchorID>\\d)", boost::regex::normal);


evb1000_driver::TREK1000::TREK1000(ros::NodeHandle &_nh) : Base1000(_nh)
{
  mbA = false;
  mbTr = false;
  mbTc = false;
}

void evb1000_driver::TREK1000::init_topics()
{
  // Tag to Anchor[0-2] distance raw
  mPubTag_raw = mNh.advertise<evb1000_driver::TagDistance>("/tagDistance_raw", 1);
  // Tag to Anchor[0-2] bias corrected distance
  mPubTag_cor = mNh.advertise<evb1000_driver::TagDistance>("/tagDistance_corrected", 1);
  // Anchor to Anchor distance
  mPubAnchor = mNh.advertise<evb1000_driver::AnchorDistance>("/anchorDistance", 1);

}

bool evb1000_driver::TREK1000::parse_data(const std::string &rawline)
{
  mbA = false;
  mbTr = false;
  mbTc = false;
  boost::smatch element;

  if(boost::regex_search(rawline, element, pattern_trek1000))
  {

    // mr -> tag to anchors raw distances,
    // mc -> tag to anchors bias corrected distances
    if(element["mid"].compare("mr") || element["mid"].compare("mc"))
    {
      evb1000_driver::TagDistance msg;
      msg.header.stamp = ros::Time::now();
      unsigned int mask = hexToUint(element["mask"]);

      if(1 & mask)
      {
        msg.distance[0] = hexToMeter(element["range0"]);
        msg.valid[0] = true;
      }
      else
      {
        msg.distance[0] = 0;
        msg.valid[0] = false;
      }
      if(2 & mask)
      {
        msg.distance[1] = hexToMeter(element["range1"]);
        msg.valid[1] = true;
      }
      else
      {
        msg.distance[1] = 0;
        msg.valid[1] = false;
      }
      if(4 & mask)
      {
        msg.distance[2] = hexToMeter(element["range2"]);
        msg.valid[2] = true;
      }
      else
      {
        msg.distance[2] = 0;
        msg.valid[2] = false;
      }
      if(8 & mask)
      {
        msg.distance[3] = hexToMeter(element["range3"]);
        msg.valid[3] = true;
      }
      else
      {
        msg.distance[3] = 0;
        msg.valid[3] = false;
      }
      msg.header.seq = hexToUint(element["rseq"]);

      if(element["mid"].compare("mr"))
      {
        mTagRawMsg = msg;
        mbTr = true;
      }
      else
      {
        mTagCorMsg = msg;
        mbTc = true;
      }
    }

    // ma -> anchor to anchor bias corrected ranges
    if(element["mid"].compare("ma"))
    {
      evb1000_driver::AnchorDistance msg;
      msg.header.stamp = ros::Time::now();
      unsigned int mask = hexToUint(element["mask"]);

      if(2 & mask)
      {
        msg.distance[0] = hexToMeter(element["range1"]);
        msg.valid[0] = true;
      }
      else
      {
        msg.distance[0] = 0;
        msg.valid[0] = false;
      }
      if(4 & mask)
      {
        msg.distance[1] = hexToMeter(element["range2"]);
        msg.valid[1] = true;
      }
      else
      {
        msg.distance[1] = 0;
        msg.valid[1] = false;
      }
      if(8 & mask)
      {
        msg.distance[2] = hexToMeter(element["range3"]);
        msg.valid[2] = true;
      }
      else
      {
        msg.distance[2] = 0;
        msg.valid[2] = false;
      }
      msg.header.seq = hexToUint(element["rseq"]);
      mAnchorMsg = msg;
      mbA = true;
    }
  }

  return (mbA || mbTc || mbTr);
}

void evb1000_driver::TREK1000::print_header(std::ofstream &file)
{
  file << "# v1.0 firmware: DecaRangeRTLS ARM Source Code v2.6" << std::endl;
  file << "# timestamp[sec] distance0[m] distance1[m] distance2[m] {distance 3[m]} seq_num type" << std::endl;
}

void evb1000_driver::TREK1000::dump_to_file(std::ofstream &file)
{
  if(mbA)
  {
    file << mAnchorMsg.header.stamp.toSec() << " "
         << mAnchorMsg.distance[0] << " " << mAnchorMsg.distance[1] << "  " << mAnchorMsg.distance[2] << " "
         << mAnchorMsg.header.seq << " anchor" <<std::endl;

  }
  if(mbTc)
  {
    file << mTagCorMsg.header.stamp.toSec() << " "
         << mTagCorMsg.distance[0] << " " << mTagCorMsg.distance[1] << "  " << mTagCorMsg.distance[2] << " " << mTagCorMsg.distance[3] << "  "
         << " " << mTagCorMsg.header.seq << " tag_cor"<<std::endl;

  }
  if(mbTr)
  {
    file << mTagRawMsg.header.stamp.toSec()  << " "
         << mTagRawMsg.distance[0] << " " << mTagRawMsg.distance[1] << "  " << mTagRawMsg.distance[2] << " " << mTagRawMsg.distance[3] << "  "
         << " " << mTagRawMsg.header.seq << " tag_raw"  <<std::endl;
  }
}

void evb1000_driver::TREK1000::publish_data()
{
  if(mbA)
  {
    mPubAnchor.publish(mAnchorMsg);
  }
  if(mbTc)
  {
    mPubTag_cor.publish(mTagCorMsg);
  }
  if(mbTr)
  {
    mPubTag_raw.publish(mTagRawMsg);
  }
}
