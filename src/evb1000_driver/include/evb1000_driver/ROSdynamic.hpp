/**
 * Copyright (C) 2016 by Austrian Institute of Technology
 *
 * @file  ROSdynamic.hpp
 *
 * @brief Automatically binds the dynamic reconfig server with the concrete callback.
 *        The user needs to implement update_(...) which updates the internal states.
 *
 * @author  Roland Jung (Roland.Jung@aau.at)
 *
 * @date  19.10.2016
 */
#ifndef ROSDYNAMIC_HPP
#define ROSDYNAMIC_HPP

#include <dynamic_reconfigure/server.h>

template < typename T>
class ROSdynamic
{
  public:

    ROSdynamic()
    {
      mFunc = boost::bind(&ROSdynamic::callback, this, _1, _2);
      mServer.setCallback(mFunc);
    }

    void callback(T &config, uint32_t level)
    {
      ROS_INFO("received reconfigure request:");
      mDynConfig = config; // read
      if(!update_() ) // modify
      {
        ROS_INFO("--> failed!");
      }
      config = mDynConfig; // write
    }

    virtual bool update_()
    {
      return false;
    }


  protected:
    dynamic_reconfigure::Server<T> mServer;
    typename dynamic_reconfigure::Server<T>::CallbackType mFunc;

    T mDynConfig;
};



#endif // ROSDYNAMIC_HPP
