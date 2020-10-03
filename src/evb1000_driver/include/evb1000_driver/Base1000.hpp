/******************************************************************************
* FILENAME:     Base1000.hpp
* PURPOSE:      Base1000
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     22.02.2018
*
*  Copyright (C) 2018
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef EVB1000_DRIVER_BASE1000_HPP
#define EVB1000_DRIVER_BASE1000_HPP
#include <iostream>
#include <fstream>
#include <string>

#include <ros_common/INode.hpp>
#include <evb1000_driver/ROSdynamic.hpp>
#include <evb1000_driver/dyn_configConfig.h>
typedef evb1000_driver::dyn_configConfig Config;

namespace evb1000_driver
{

  unsigned int hexToUint(std::string const& inp);
  float hexToMeter(std::string  const& inp);


  class Base1000 : public ros_common::INode, public ROSdynamic<Config>
  {
    public:
      Base1000(ros::NodeHandle & _nh);

    protected:
      virtual bool parse_data(std::string const& rawline) = 0;
      virtual void print_header(std::ofstream& file) = 0;
      virtual void dump_to_file(std::ofstream& file) = 0;
      virtual void publish_data() = 0;

      bool mbVerbose = false;
    private:
      // DERIVED METHODS:
      bool update_() override final;
      void load_config();
      void init() override final;
      void process()  override final;   // read line +  parse + publish + dump
      //virtual void init_topics() = 0;

      std::string serial_device = "/dev/ttyACM0";
      std::ifstream device;
      std::ofstream file;
      std::atomic<bool> mbRecording;
  };
}


#endif // EVB1000_DRIVER_BASE1000_HPP
