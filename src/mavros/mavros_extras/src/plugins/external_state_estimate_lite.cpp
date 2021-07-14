/**
 * @brief External state estimate plugin
 * @file external_state_estimate.cpp
 * @author Alessandro Fornasier <alessandro.fornasier@aau.at>
 * @author Christian Brommer <christian.brommer@aau.at>
 *
 * Send external state estimator estimation to FCU
 *
 */

#include <Eigen/Dense>
#include <mavros/mavros_plugin.h>
#include <mavros_msgs/ExtCoreStateLite.h>
#include <pluginlib/class_list_macros.h>
#include <string>

namespace mavros {
namespace extra_plugins {

class ExternalStateEstimateLitePlugin : public plugin::PluginBase {

public:
  ExternalStateEstimateLitePlugin() : PluginBase(), nh("~external_state_lite") {}

  void initialize(UAS &uas_) {
    PluginBase::initialize(uas_);
    std::string topic = "external_state_estimate_lite";
    nh.param<std::string>("external_state_estimate_lite", topic, topic);
    sub = nh.subscribe(topic, 1, &ExternalStateEstimateLitePlugin::ext_state_lite_cb,
                       this);
  }

  Subscriptions get_subscriptions() { return {/* Rx disabled */}; }

private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  uint64_t last_transform_stamp_usec;

  /* -*- low-level send -*- */
  void send_estimate(const uint64_t usec, const Eigen::Vector3d &pos,
                     const Eigen::Vector3d &vel,
                     const Eigen::Quaterniond &att) {
    if (last_transform_stamp_usec == usec) {
      ROS_WARN("Discarded message with same timestamp");
      return;
    }
    last_transform_stamp_usec = usec;

    //<message id="6000" name="EXT_CORE_STATE">
    //      <description>external core state</description>
    //      <field type="uint64_t" name="usec" units="us">Timestamp
    //      (UNIX)</field>
    //      <field type="float[3]" name="p_wi">position x y z of i expressed in
    //      w (NED)</field>
    //      <field type="float[4]" name="q_wi">orientation qx qy qz qw from i to
    //      w (NED) in Hamiltonian convention</field>
    //      <field type="float[3]" name="v_wi">velocity vx vy vz of i expressed
    //      in w (NED)</field>
    //</message>

    mavlink::common::msg::EXT_CORE_STATE_LITE ecsl;
    ecsl.usec = usec;
    Eigen::Map<Eigen::Vector3f>(ecsl.p_wi.data()) = pos.cast<float>();
    Eigen::Map<Eigen::Vector3f>(ecsl.v_wi.data()) = vel.cast<float>();
    ftf::quaternion_to_mavlink(att, ecsl.q_wi);
    UAS_FCU(m_uas)->send_message_ignore_drop(ecsl);
  }

  //	/* -*- callbacks -*- */
  void ext_state_lite_cb(const mavros_msgs::ExtCoreStateLite::ConstPtr &msg) {
    Eigen::Vector3d pos(msg->p_wi.x, msg->p_wi.y, msg->p_wi.z);
    Eigen::Vector3d vel(msg->v_wi.x, msg->v_wi.y, msg->v_wi.z);
    Eigen::Vector4d att_coeffs(msg->q_wi.x, msg->q_wi.y, msg->q_wi.z,
                               msg->q_wi.w);
    Eigen::Quaterniond att;
    uint64_t usec = msg->header.stamp.toNSec() / 1000;

    switch (msg->QUATERNION_TYPE) {
    case 0:
      ROS_WARN("Discarded message with unkown quaternion type");
      return;
    case 1:
      att = Eigen::Quaterniond(att_coeffs(3), att_coeffs(0), att_coeffs(1),
                               att_coeffs(2));
      break;
    case 2:
      // Alessandro: Converts JPL quaterion to SO(3) rotation matrix and then
      // back again to Hamiltonian quaternion
      Eigen::Matrix3d skew;
      skew << 0, -att_coeffs(2), att_coeffs(1), att_coeffs(2), 0,
          -att_coeffs(0), -att_coeffs(1), att_coeffs(0), 0;
      Eigen::Matrix3d Rot =
          (2 * std::pow(att_coeffs(3), 2) - 1) * Eigen::Matrix3d::Identity() -
          2 * att_coeffs(3) * skew +
          2 * att_coeffs.segment(0, 3) * att_coeffs.segment(0, 3).transpose();
      att = Eigen::Quaterniond(Rot);
      break;
    }

    switch (msg->FRAME_TYPE) {
    case 0:
      ROS_WARN("Discarded message with unkown frame type");
      return;
    case 1:
      pos = ftf::transform_frame_enu_ned(Eigen::Vector3d(pos));
      vel = ftf::transform_frame_enu_ned(Eigen::Vector3d(vel));
      // Alessandro: Potential shit! check if baselink_aircraft conversion is
      // needed
      att = ftf::transform_orientation_enu_ned(
          ftf::transform_orientation_baselink_aircraft(
              Eigen::Quaterniond(att)));
      // att = ftf::transform_orientation_enu_ned(Eigen::Quaterniond(att));
      break;
    case 2:
      // Alessandro: Potential shit! check if baselink_aircraft conversion is
      // needed
      att =
          ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(att));
      break;
    }

    send_estimate(usec, pos, vel, att);
  }

};

} // namespace extra_plugins
} // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ExternalStateEstimateLitePlugin,
                       mavros::plugin::PluginBase)
