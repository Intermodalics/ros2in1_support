/******************************************************************************
 *  Copyright (c) 2024, Intermodalics BVBA                                    *
 *  All rights reserved.                                                      *
 ******************************************************************************/

#ifndef ROS2IN1_SUPPORT_CONVERSIONS_STD_MSGS_H
#define ROS2IN1_SUPPORT_CONVERSIONS_STD_MSGS_H
#ifdef ROS2_SUPPORT

#include <utility>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/Header.h>

#include "builtin_interfaces.h"

namespace ros2in1_support {
namespace conversions {

template <typename Ros1MessageT> struct Ros2MessageType;
template <> struct Ros2MessageType<std_msgs::Header> {
  typedef std_msgs::msg::Header type;
};

template <>
inline void convert_2_to_1<std_msgs::Header, std_msgs::msg::Header>(
    const std_msgs::msg::Header& ros2_msg,
    std_msgs::Header& ros1_msg) {
  convert_2_to_1<ros::Time, builtin_interfaces::msg::Time>(ros2_msg.stamp, ros1_msg.stamp);
  ros1_msg.frame_id = ros2_msg.frame_id;
}

template <>
inline void convert_2_to_1<std_msgs::Header, std_msgs::msg::Header>(
    std_msgs::msg::Header&& ros2_msg,
    std_msgs::Header& ros1_msg) {
  convert_2_to_1<ros::Time, builtin_interfaces::msg::Time>(ros2_msg.stamp, ros1_msg.stamp);
  ros1_msg.frame_id = std::move(ros2_msg.frame_id);
}

template <>
inline void convert_1_to_2<std_msgs::msg::Header, std_msgs::Header>(
    const std_msgs::Header& ros1_msg,
    std_msgs::msg::Header& ros2_msg) {
  convert_1_to_2<builtin_interfaces::msg::Time, ros::Time>(ros1_msg.stamp, ros2_msg.stamp);
  ros2_msg.frame_id = ros1_msg.frame_id;
}

template <>
inline void convert_1_to_2<std_msgs::msg::Header, std_msgs::Header>(
    std_msgs::Header&& ros1_msg,
    std_msgs::msg::Header& ros2_msg) {
  convert_1_to_2<builtin_interfaces::msg::Time, ros::Time>(ros1_msg.stamp, ros2_msg.stamp);
  ros2_msg.frame_id = std::move(ros1_msg.frame_id);
}

}  // namespace conversions
}  // namespace ros2in1_support

#endif  // ROS2_SUPPORT
#endif  // ROS2IN1_SUPPORT_CONVERSIONS_STD_MSGS_H
