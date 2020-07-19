/**
 * @licence MIT
 *
 * @file  INode.hpp
 *
 * @brief  refactored pfe_node.h
 *
 * @author  Roland Jung
 *
 * @date  14.12.2016
 */
#ifndef ROS_COMMON_INODE_HPP
#define ROS_COMMON_INODE_HPP
#include <cassert>
#include <ros/ros.h>
#include <cstdint>
#include <atomic>

using std::uint32_t;
using std::uint16_t;
using std::int16_t;

namespace ros_common
{

  /**
   * abstract baseclass for node with custom spinning
   * order of execution of virtual methods:
   *  1) init_topics() subscribe and publish topics, initialize message independet stuff
   *  1.1) print_topic_info()
   *  2) load_config()
   *  3) init() called after first message is arrived
   *  4) process() called with each message
   *
   *  signal run() that new data is arrived with the mbHasNewMessage flag
   *
   *  params:
   *   node_profile: enable timing mesauremnet for process() and overhead
   *   node_spinning_rate: desired spinning rate [Hz]. default 50 Hz
   *   node_warn_time: warn if no message arrived in X s. default 15 (0 do disable)
   *
   */
  class INode
  {
    protected:
      INode(ros::NodeHandle &nh) : mNh(nh)
      {
        mbIsInitialized = false;
        mbHasNewMessage = false;
        mbSpinningProcess = false;
        mbProfile = true;
      }

    public:
      void run();

    protected:

      /**
       * 1) subscribe and publish topics, initialize message independet stuff
       */
      virtual void init_topics() = 0;

      /**
       * 2) loading private ROS parameters
       */
      virtual void load_config() = 0;

      /**
         * 3) initialize node; called after first message received
         */
      virtual void init() = 0;

      /**
         * 4) process message. called when mbHasNewMessage or mbSpinningProcess was set
         */
      virtual void process() = 0;


      void get_param_u32(std::string const &key, std::uint32_t &value, std::uint32_t const &def);

      void get_param_s16(std::string const &key, std::int16_t &value, std::int16_t const &def);

      void param_get_string(std::string &variable, std::string const &param, std::string const &default_value);

      void param_get_int(int &variable, std::string const &param, int const &default_value);

      void param_get_uint(unsigned &variable, std::string const &param, unsigned const &default_value);

      void param_get_float(float &variable, std::string const &param, float const &default_value);

      template<typename T>
      void param_check_range(T &variable, std::string const &param_name, T const &min, T const &max,
                             T const &default_value)
      {
        if(variable < min || variable > max)
        {
          variable = default_value;
          ROS_WARN_STREAM(param_name << " invalid value (" << variable << ")! Value set to " << default_value);
        }
      }

      /// MEMBERS:

      ros::NodeHandle mNh;


      std::atomic<bool> mbHasNewMessage;
      std::atomic<bool> mbSpinningProcess;  // for a node that does the process loop iteratively without new messages
      float mPeriod_s;


    private:
      bool mbProfile;
      bool mbIsInitialized;

      INode();

      INode(const INode &);

      INode &operator=(const INode &);

      /**
       * Print all subscritpions and advertations
       */
      void print_topic_info();

  };

} // namespace ros_common

#endif // ROS_COMMON_INODE_HPP
