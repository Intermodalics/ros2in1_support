/******************************************************************************
 *  Copyright (c) 2024, Intermodalics BVBA                                    *
 *  All rights reserved.                                                      *
 ******************************************************************************/

#ifndef ROS2IN1_SUPPORT_CONVERSIONS_BUILTIN_INTERFACES_H
#define ROS2IN1_SUPPORT_CONVERSIONS_BUILTIN_INTERFACES_H
#ifdef ROS2_SUPPORT

#include <builtin_interfaces/msg/time.hpp>
#include <ros/time.h>

#include "common.h"

namespace ros2in1_support {
namespace conversions {

template <typename Ros1MessageT> struct Ros2MessageType;
template <> struct Ros2MessageType<ros::Time> {
  typedef builtin_interfaces::msg::Time type;
};

template <>
inline void convert_2_to_1<ros::Time, builtin_interfaces::msg::Time>(
    const builtin_interfaces::msg::Time& ros2_msg,
    ros::Time& ros1_msg) {
  ros1_msg.sec = static_cast<uint32_t>(ros2_msg.sec);
  ros1_msg.nsec = ros2_msg.nanosec;
}

template <>
inline void convert_1_to_2<builtin_interfaces::msg::Time, ros::Time>(
    const ros::Time& ros1_msg,
    builtin_interfaces::msg::Time& ros2_msg) {
  ros2_msg.sec = static_cast<int32_t>(ros1_msg.sec);
  ros2_msg.nanosec = ros1_msg.nsec;
}

}  // namespace conversions
}  // namespace ros2in1_support

#endif  // ROS2_SUPPORT
#endif  // ROS2IN1_SUPPORT_CONVERSIONS_BUILTIN_INTERFACES_H
