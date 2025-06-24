/******************************************************************************
 *  Copyright (c) 2024, Intermodalics BVBA                                    *
 *  All rights reserved.                                                      *
 ******************************************************************************/

#ifndef ROS2IN1_SUPPORT_CONVERSIONS_GEOMETRY_MSGS_H
#define ROS2IN1_SUPPORT_CONVERSIONS_GEOMETRY_MSGS_H
#ifdef ROS2_SUPPORT

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "common.h"
#include "std_msgs.h"

namespace ros2in1_support {
namespace conversions {

template <typename Ros1MessageT> struct Ros2MessageType;
template <> struct Ros2MessageType<geometry_msgs::Point> {
  typedef geometry_msgs::msg::Point type;
};
template <> struct Ros2MessageType<geometry_msgs::PointStamped> {
  typedef geometry_msgs::msg::PointStamped type;
};
template <> struct Ros2MessageType<geometry_msgs::Pose> {
  typedef geometry_msgs::msg::Pose type;
};
template <> struct Ros2MessageType<geometry_msgs::PoseArray> {
  typedef geometry_msgs::msg::PoseArray type;
};
template <> struct Ros2MessageType<geometry_msgs::PoseStamped> {
  typedef geometry_msgs::msg::PoseStamped type;
};
template <> struct Ros2MessageType<geometry_msgs::Quaternion> {
  typedef geometry_msgs::msg::Quaternion type;
};
template <> struct Ros2MessageType<geometry_msgs::Transform> {
  typedef geometry_msgs::msg::Transform type;
};
template <> struct Ros2MessageType<geometry_msgs::TransformStamped> {
  typedef geometry_msgs::msg::TransformStamped type;
};
template <> struct Ros2MessageType<geometry_msgs::Twist> {
  typedef geometry_msgs::msg::Twist type;
};
template <> struct Ros2MessageType<geometry_msgs::Vector3> {
  typedef geometry_msgs::msg::Vector3 type;
};
template <> struct Ros2MessageType<geometry_msgs::Vector3Stamped> {
  typedef geometry_msgs::msg::Vector3Stamped type;
};

template <>
inline void convert_2_to_1<geometry_msgs::Point, geometry_msgs::msg::Point>(
    const geometry_msgs::msg::Point& ros2_msg,
    geometry_msgs::Point& ros1_msg) {
  ros1_msg.x = ros2_msg.x;
  ros1_msg.y = ros2_msg.y;
  ros1_msg.z = ros2_msg.z;
}

template <>
inline void convert_1_to_2<geometry_msgs::msg::Point, geometry_msgs::Point>(
    const geometry_msgs::Point& ros1_msg,
    geometry_msgs::msg::Point& ros2_msg) {
  ros2_msg.x = ros1_msg.x;
  ros2_msg.y = ros1_msg.y;
  ros2_msg.z = ros1_msg.z;
}

template <>
inline void convert_2_to_1<geometry_msgs::PointStamped, geometry_msgs::msg::PointStamped>(
    const geometry_msgs::msg::PointStamped& ros2_msg,
    geometry_msgs::PointStamped& ros1_msg) {
  convert_2_to_1(ros2_msg.header, ros1_msg.header);
  convert_2_to_1(ros2_msg.point, ros1_msg.point);
}

template <>
inline void convert_1_to_2<geometry_msgs::msg::PointStamped, geometry_msgs::PointStamped>(
    const geometry_msgs::PointStamped& ros1_msg,
    geometry_msgs::msg::PointStamped& ros2_msg) {
  convert_1_to_2(ros1_msg.header, ros2_msg.header);
  convert_1_to_2(ros1_msg.point, ros2_msg.point);
}

template <>
inline void convert_2_to_1<geometry_msgs::Quaternion, geometry_msgs::msg::Quaternion>(
    const geometry_msgs::msg::Quaternion& ros2_msg,
    geometry_msgs::Quaternion& ros1_msg) {
  ros1_msg.x = ros2_msg.x;
  ros1_msg.y = ros2_msg.y;
  ros1_msg.z = ros2_msg.z;
  ros1_msg.w = ros2_msg.w;
}

template <>
inline void convert_1_to_2<geometry_msgs::msg::Quaternion, geometry_msgs::Quaternion>(
    const geometry_msgs::Quaternion& ros1_msg,
    geometry_msgs::msg::Quaternion& ros2_msg) {
  ros2_msg.x = ros1_msg.x;
  ros2_msg.y = ros1_msg.y;
  ros2_msg.z = ros1_msg.z;
  ros2_msg.w = ros1_msg.w;
}

template <>
inline void convert_2_to_1<geometry_msgs::Pose, geometry_msgs::msg::Pose>(
    const geometry_msgs::msg::Pose& ros2_msg,
    geometry_msgs::Pose& ros1_msg) {
  convert_2_to_1(ros2_msg.position, ros1_msg.position);
  convert_2_to_1(ros2_msg.orientation, ros1_msg.orientation);
}

template <>
inline void convert_1_to_2<geometry_msgs::msg::Pose, geometry_msgs::Pose>(
    const geometry_msgs::Pose& ros1_msg,
    geometry_msgs::msg::Pose& ros2_msg) {
  convert_1_to_2(ros1_msg.position, ros2_msg.position);
  convert_1_to_2(ros1_msg.orientation, ros2_msg.orientation);
}

template <>
inline void convert_2_to_1<geometry_msgs::PoseArray, geometry_msgs::msg::PoseArray>(
    const geometry_msgs::msg::PoseArray& ros2_msg,
    geometry_msgs::PoseArray& ros1_msg) {
  convert_2_to_1(ros2_msg.header, ros1_msg.header);
  convert_2_to_1(ros2_msg.poses, ros1_msg.poses);
}

template <>
inline void convert_1_to_2<geometry_msgs::msg::PoseArray, geometry_msgs::PoseArray>(
    const geometry_msgs::PoseArray& ros1_msg,
    geometry_msgs::msg::PoseArray& ros2_msg) {
  convert_1_to_2(ros1_msg.header, ros2_msg.header);
  convert_1_to_2(ros1_msg.poses, ros2_msg.poses);
}

template <>
inline void convert_2_to_1<geometry_msgs::PoseStamped, geometry_msgs::msg::PoseStamped>(
    const geometry_msgs::msg::PoseStamped& ros2_msg,
    geometry_msgs::PoseStamped& ros1_msg) {
  convert_2_to_1(ros2_msg.header, ros1_msg.header);
  convert_2_to_1(ros2_msg.pose, ros1_msg.pose);
}

template <>
inline void convert_1_to_2<geometry_msgs::msg::PoseStamped, geometry_msgs::PoseStamped>(
    const geometry_msgs::PoseStamped& ros1_msg,
    geometry_msgs::msg::PoseStamped& ros2_msg) {
  convert_1_to_2(ros1_msg.header, ros2_msg.header);
  convert_1_to_2(ros1_msg.pose, ros2_msg.pose);
}

template <>
inline void convert_2_to_1<geometry_msgs::Vector3, geometry_msgs::msg::Vector3>(
    const geometry_msgs::msg::Vector3& ros2_msg,
    geometry_msgs::Vector3& ros1_msg) {
  ros1_msg.x = ros2_msg.x;
  ros1_msg.y = ros2_msg.y;
  ros1_msg.z = ros2_msg.z;
}

template <>
inline void convert_1_to_2<geometry_msgs::msg::Vector3, geometry_msgs::Vector3>(
    const geometry_msgs::Vector3& ros1_msg,
    geometry_msgs::msg::Vector3& ros2_msg) {
  ros2_msg.x = ros1_msg.x;
  ros2_msg.y = ros1_msg.y;
  ros2_msg.z = ros1_msg.z;
}

template <>
inline void convert_2_to_1<geometry_msgs::Vector3Stamped, geometry_msgs::msg::Vector3Stamped>(
    const geometry_msgs::msg::Vector3Stamped& ros2_msg,
    geometry_msgs::Vector3Stamped& ros1_msg) {
  convert_2_to_1(ros2_msg.header, ros1_msg.header);
  convert_2_to_1(ros2_msg.vector, ros1_msg.vector);
}

template <>
inline void convert_1_to_2<geometry_msgs::msg::Vector3Stamped, geometry_msgs::Vector3Stamped>(
    const geometry_msgs::Vector3Stamped& ros1_msg,
    geometry_msgs::msg::Vector3Stamped& ros2_msg) {
  convert_1_to_2(ros1_msg.header, ros2_msg.header);
  convert_1_to_2(ros1_msg.vector, ros2_msg.vector);
}

template <>
inline void convert_2_to_1<geometry_msgs::Transform, geometry_msgs::msg::Transform>(
    const geometry_msgs::msg::Transform& ros2_msg,
    geometry_msgs::Transform& ros1_msg) {
  convert_2_to_1(ros2_msg.translation, ros1_msg.translation);
  convert_2_to_1(ros2_msg.rotation, ros1_msg.rotation);
}

template <>
inline void convert_1_to_2<geometry_msgs::msg::Transform, geometry_msgs::Transform>(
    const geometry_msgs::Transform& ros1_msg,
    geometry_msgs::msg::Transform& ros2_msg) {
  convert_1_to_2(ros1_msg.translation, ros2_msg.translation);
  convert_1_to_2(ros1_msg.rotation, ros2_msg.rotation);
}

template <>
inline void convert_2_to_1<geometry_msgs::TransformStamped, geometry_msgs::msg::TransformStamped>(
    const geometry_msgs::msg::TransformStamped& ros2_msg,
    geometry_msgs::TransformStamped& ros1_msg) {
  convert_2_to_1(ros2_msg.header, ros1_msg.header);
  ros1_msg.child_frame_id = ros2_msg.child_frame_id;
  convert_2_to_1(ros2_msg.transform, ros1_msg.transform);
}

template <>
inline void convert_1_to_2<geometry_msgs::msg::TransformStamped, geometry_msgs::TransformStamped>(
    const geometry_msgs::TransformStamped& ros1_msg,
    geometry_msgs::msg::TransformStamped& ros2_msg) {
  convert_1_to_2(ros1_msg.header, ros2_msg.header);
  ros2_msg.child_frame_id = ros1_msg.child_frame_id;
  convert_1_to_2(ros1_msg.transform, ros2_msg.transform);
}

template <>
inline void convert_2_to_1<geometry_msgs::Twist, geometry_msgs::msg::Twist>(
    const geometry_msgs::msg::Twist& ros2_msg,
    geometry_msgs::Twist& ros1_msg) {
  convert_2_to_1<geometry_msgs::Vector3, geometry_msgs::msg::Vector3>(ros2_msg.linear, ros1_msg.linear);
  convert_2_to_1<geometry_msgs::Vector3, geometry_msgs::msg::Vector3>(ros2_msg.angular, ros1_msg.angular);
}

template <>
inline void convert_2_to_1<geometry_msgs::Twist, geometry_msgs::msg::Twist>(
    geometry_msgs::msg::Twist&& ros2_msg,
    geometry_msgs::Twist& ros1_msg) {
  convert_2_to_1<geometry_msgs::Vector3, geometry_msgs::msg::Vector3>(ros2_msg.linear, ros1_msg.linear);
  convert_2_to_1<geometry_msgs::Vector3, geometry_msgs::msg::Vector3>(ros2_msg.angular, ros1_msg.angular);
}

template <>
inline void convert_1_to_2<geometry_msgs::msg::Twist, geometry_msgs::Twist>(
    const geometry_msgs::Twist& ros1_msg,
    geometry_msgs::msg::Twist& ros2_msg) {
  convert_1_to_2(ros1_msg.linear, ros2_msg.linear);
  convert_1_to_2(ros1_msg.angular, ros2_msg.angular);
}

}  // namespace conversions
}  // namespace ros2in1_support

#endif  // ROS2_SUPPORT
#endif  // ROS2IN1_SUPPORT_CONVERSIONS_GEOMETRY_MSGS_H
