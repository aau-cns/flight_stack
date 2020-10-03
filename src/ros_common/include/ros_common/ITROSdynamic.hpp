/**
 * Copyright (C) 2016 by Austrian Institute of Technology
 *
 * @file  ITROSdynamic.hpp
 *
 * @brief Automatically binds the dynamic reconfig server with the concrete callback.
 *        The user needs to implement update_(...) which updates the internal states.
 *
 * @author  Roland Jung (Roland.Jung@aau.at)
 *
 * @date  19.10.2016
 */
#ifndef ROS_COMMON_ITROSDYNAMIC_HPP
#define ROS_COMMON_ITROSDYNAMIC_HPP

#include <dynamic_reconfigure/server.h>

namespace ros_common
{

  /**
     * abstract template class
     * Starts a dynamic reconfigure server for the configuration T.
     * On a change the callback will store the latest value in mDynConfig
     * and call the abstract update_() method in order to notify the child class
     */
  template < typename T>
  class ITROSdynamic
  {
    public:

      ITROSdynamic()
      {
        mFunc = boost::bind(&ITROSdynamic::callback, this, _1, _2);
        mServer.setCallback(mFunc);
      }

      void callback(T &config, uint32_t level)
      {
        ROS_INFO("received reconfigure request:");
        mDynConfig = config;  //read
        if(!update_() )       // modify
        {
          ROS_INFO("--> failed!");
        }
        config = mDynConfig;  // write
      }

      virtual bool update_()
      {
        return true;
      }


    protected:
      dynamic_reconfigure::Server<T> mServer;
      typename dynamic_reconfigure::Server<T>::CallbackType mFunc;

      T mDynConfig;

  }; // class ITROSdynamic

} // namespace ros_common


#endif // ROS_COMMON_ITROSDYNAMIC_HPP
