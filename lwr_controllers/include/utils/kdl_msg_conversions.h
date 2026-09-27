// Replacement of the ROS 1 kdl_conversions / tf_conversions functions used by the controllers

#ifndef KDL_MSG_CONVERSIONS_H
#define KDL_MSG_CONVERSIONS_H

#include <kdl/frames.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>

namespace tf
{
  inline void wrenchKDLToMsg(const KDL::Wrench &k, geometry_msgs::msg::Wrench &m)
  {
    m.force.x = k.force.x();
    m.force.y = k.force.y();
    m.force.z = k.force.z();
    m.torque.x = k.torque.x();
    m.torque.y = k.torque.y();
    m.torque.z = k.torque.z();
  }

  inline void wrenchMsgToKDL(const geometry_msgs::msg::Wrench &m, KDL::Wrench &k)
  {
    k.force = KDL::Vector(m.force.x, m.force.y, m.force.z);
    k.torque = KDL::Vector(m.torque.x, m.torque.y, m.torque.z);
  }

  inline void poseKDLToMsg(const KDL::Frame &k, geometry_msgs::msg::Pose &m)
  {
    m.position.x = k.p.x();
    m.position.y = k.p.y();
    m.position.z = k.p.z();
    k.M.GetQuaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w);
  }

  inline void poseMsgToKDL(const geometry_msgs::msg::Pose &m, KDL::Frame &k)
  {
    k.p = KDL::Vector(m.position.x, m.position.y, m.position.z);
    k.M = KDL::Rotation::Quaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w);
  }

  inline void transformMsgToKDL(const geometry_msgs::msg::TransformStamped &m, KDL::Frame &k)
  {
    k.p = KDL::Vector(m.transform.translation.x, m.transform.translation.y, m.transform.translation.z);
    k.M = KDL::Rotation::Quaternion(m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w);
  }
}

#endif
