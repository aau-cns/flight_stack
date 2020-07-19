/******************************************************************************
* FILENAME:     EVK1000.hpp
* PURPOSE:      TREK1000
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     22.02.2018
*
*  Copyright (C) 2018
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef EVB1000_DRIVER_EVK1000_HPP
#define EVB1000_DRIVER_EVK1000_HPP


#include <evb1000_driver/Base1000.hpp>
#include <evb1000_driver/RangingMeasurement.h>

namespace evb1000_driver
{

  class EVK1000 : public Base1000
  {
    public:
      EVK1000(ros::NodeHandle & _nh);

    private:
      // Node interface
      void init_topics();
      // Base1000 interface
      bool parse_data(const std::string &rawline) override final;
      void print_header(std::ofstream& file) override final;
      void dump_to_file(std::ofstream& file)  override final;
      void publish_data()  override final;


      ros::Publisher mPubRangeMeasurement;
      evb1000_driver::RangingMeasurement mMsg;
  };
}

#endif // EVB1000_DRIVER_EVK1000_HPP
