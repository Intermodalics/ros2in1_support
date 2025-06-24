/******************************************************************************
 *  Copyright (c) 2025, Intermodalics BVBA                                    *
 *  All rights reserved.                                                      *
 ******************************************************************************/

#ifndef ROS2IN1_SUPPORT_CONVERSIONS_STD_SRVS_H
#define ROS2IN1_SUPPORT_CONVERSIONS_STD_SRVS_H
#ifdef ROS2_SUPPORT

#include <utility>

#include <std_srvs/srv/empty.hpp>
#include <std_srvs/Empty.h>

#include "builtin_interfaces.h"

namespace ros2in1_support {
namespace conversions {

template <typename Ros1ServiceT> struct Ros2ServiceType;

template <> struct Ros2ServiceType<std_srvs::Empty> {
  typedef std_srvs::srv::Empty type;
};

template <>
inline void convert_2_to_1<std_srvs::Empty::Request, std_srvs::srv::Empty::Request>(
    const std_srvs::srv::Empty::Request&,
    std_srvs::Empty::Request&) {
  // empty
}

template <>
inline void convert_1_to_2<std_srvs::srv::Empty::Response, std_srvs::Empty::Response>(
    const std_srvs::Empty::Response&,
    std_srvs::srv::Empty::Response&) {
  // empty
}

template <>
inline void convert_1_to_2<std_srvs::srv::Empty::Response, std_srvs::Empty::Response>(
    std_srvs::Empty::Response&&,
    std_srvs::srv::Empty::Response&) {
  // empty
}

}  // namespace conversions
}  // namespace ros2in1_support

#endif  // ROS2_SUPPORT
#endif  // ROS2IN1_SUPPORT_CONVERSIONS_STD_SRVS_H
