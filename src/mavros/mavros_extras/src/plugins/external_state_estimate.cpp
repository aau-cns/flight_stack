/**
 * @brief External state estimate plugin
 * @file external_state_estimate.cpp
 * @author Alessandro Fornasier <alessandro.fornasier@aau.at>
 * @author Christian Brommer <christian.brommer@aau.at>
 *
 * Send external state estimator estimation to FCU
 *
 */

#include <string>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <Eigen/Dense>
#include <mavros_msgs/ExtCoreState.h>

namespace mavros {
namespace extra_plugins{

class ExternalStateEstimatePlugin : public plugin::PluginBase {

public:
  ExternalStateEstimatePlugin() :
	PluginBase(), nh("~external_state") {}

	void initialize(UAS &uas_) {
		PluginBase::initialize(uas_);
    std::string topic = "external_state_estimate";
    nh.param<std::string>("external_state_estimate", topic, topic);
		sub = nh.subscribe(topic, 1, &ExternalStateEstimatePlugin::ext_state_cb, this);
	}

	Subscriptions get_subscriptions() {
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle nh;
	ros::Subscriber sub;
  uint64_t last_transform_stamp_usec;

	/* -*- low-level send -*- */
  void send_estimate(const uint64_t usec, const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const Eigen::Quaterniond &att, const Eigen::Vector3d &ba, const Eigen::Vector3d &bw, const Eigen::Matrix<double,9,9> &cov_urt)
	{
    if (last_transform_stamp_usec == usec) {
      ROS_WARN("Discarded message with same timestamp");
      return;
    }
    last_transform_stamp_usec = usec;

		//<message id="6000" name="EXT_CORE_STATE">
		//      <description>external core state</description>
		//      <field type="uint64_t" name="usec" units="us">Timestamp (UNIX)</field>
		//      <field type="float[3]" name="p_wi">position x y z of i expressed in w (NED)</field>
    //      <field type="float[4]" name="q_wi">orientation qx qy qz qw from i to w (NED) in Hamiltonian convention</field>
		//      <field type="float[3]" name="v_wi">velocity vx vy vz of i expressed in w (NED)</field>
		//      <field type="float[3]" name="ba">accelerometer bias</field>
		//      <field type="float[3]" name="bw">accelerometer bias</field>
		//      <field type="float[225]" name="cov">x velocity</field>
		//</message>

    mavlink::common::msg::EXT_CORE_STATE ecs;
    ecs.usec = usec;
    Eigen::Map<Eigen::Vector3f>(ecs.p_wi.data()) = pos.cast<float>();
    Eigen::Map<Eigen::Vector3f>(ecs.v_wi.data()) = vel.cast<float>();
    ftf::quaternion_to_mavlink(att, ecs.q_wi);
    Eigen::Map<Eigen::Vector3f>(ecs.b_a.data()) = ba.cast<float>();
    Eigen::Map<Eigen::Vector3f>(ecs.b_w.data()) = bw.cast<float>();
    ftf::covariance_urt_to_mavlink(cov_urt, ecs.cov_urt);
    UAS_FCU(m_uas)->send_message_ignore_drop(ecs);
  }

//	/* -*- callbacks -*- */
  void ext_state_cb(const mavros_msgs::ExtCoreState::ConstPtr &msg)
  {
    Eigen::Vector3d pos(msg->p_wi.x, msg->p_wi.y, msg->p_wi.z);
    Eigen::Vector3d vel(msg->v_wi.x, msg->v_wi.y, msg->v_wi.z);
    Eigen::Vector4d att_coeffs(msg->q_wi.x, msg->q_wi.y, msg->q_wi.z, msg->q_wi.w);
    Eigen::Quaterniond att;
    Eigen::Vector3d ba(msg->b_a.x, msg->b_a.y, msg->b_a.z);
    Eigen::Vector3d bw(msg->b_w.x, msg->b_w.y, msg->b_w.z);
    // Alessandro: just extract the position, orientation and velocity covariance
    Eigen::Matrix<double,9,9> cov_urt = (Eigen::Map<const Eigen::Matrix<double,15,15>>(msg->cov.data())).block(0,0,9,9).triangularView<Eigen::Upper>();
    uint64_t usec = msg->header.stamp.toNSec()/1000;

    switch (msg->QUATERNION_TYPE) {
    case 0:
      ROS_WARN("Discarded message with unkown quaternion type");
      return;
    case 1:
      att = Eigen::Quaterniond(att_coeffs(3), att_coeffs(0), att_coeffs(1), att_coeffs(2));
      break;
    case 2:
      // Alessandro: Converts JPL quaterion to SO(3) rotation matrix and then back again to Hamiltonian quaternion
      Eigen::Matrix3d skew;
      skew << 0, -att_coeffs(2), att_coeffs(1), att_coeffs(2), 0, -att_coeffs(0), -att_coeffs(1), att_coeffs(0), 0;
      Eigen::Matrix3d Rot = (2 * std::pow(att_coeffs(3), 2) - 1)*Eigen::Matrix3d::Identity() - 2*att_coeffs(3)*skew + 2*att_coeffs.segment(0,3)*att_coeffs.segment(0,3).transpose();
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
      ba = ftf::transform_frame_enu_ned(Eigen::Vector3d(ba));
      bw = ftf::transform_frame_enu_ned(Eigen::Vector3d(bw));
      // Alessandro: covariance conversion from enu to ned. Possibly wrong check ENU FRD
      transform_covariance_enu_ned(cov_urt);
      // Alessandro: Potential shit! check if baselink_aircraft conversion is needed
      att = ftf::transform_orientation_enu_ned(ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(att)));
      //att = ftf::transform_orientation_enu_ned(Eigen::Quaterniond(att));
      break;
    case 2:
      // Alessandro: Potential shit! check if baselink_aircraft conversion is needed
      att = ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(att));
      break;
    }

    send_estimate(usec, pos, vel, att, ba, bw, cov_urt);
	}

  void transform_covariance_enu_ned(Eigen::Matrix<double,9,9> &cov) {
    Eigen::Matrix3d T;
    T << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    Eigen::Matrix<double,9,9> F = Eigen::Matrix<double,9,9>::Zero();
    F.block(0,0,3,3) = T;
    F.block(3,3,3,3) = T;
    F.block(6,6,3,3) = T;
    cov = F*cov*F.transpose();
    return;
  }
};

}	// namespace extra_plugins
}	// namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ExternalStateEstimatePlugin, mavros::plugin::PluginBase)
