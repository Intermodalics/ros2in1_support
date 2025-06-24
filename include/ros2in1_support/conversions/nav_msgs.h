/******************************************************************************
 *  Copyright (c) 2024, Intermodalics BVBA                                    *
 *  All rights reserved.                                                      *
 ******************************************************************************/

#ifndef ROS2IN1_SUPPORT_CONVERSIONS_NAV_MSGS_H
#define ROS2IN1_SUPPORT_CONVERSIONS_NAV_MSGS_H
#ifdef ROS2_SUPPORT

#include <algorithm>

#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_map.hpp>

#include <nav_msgs/GetMap.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "common.h"
#include "geometry_msgs.h"
#include "std_msgs.h"

namespace ros2in1_support {
namespace conversions {

template <typename Ros1MessageT> struct Ros2MessageType;
template <> struct Ros2MessageType<nav_msgs::MapMetaData> {
  typedef nav_msgs::msg::MapMetaData type;
};
template <> struct Ros2MessageType<nav_msgs::OccupancyGrid> {
  typedef nav_msgs::msg::OccupancyGrid type;
};
template <> struct Ros2MessageType<nav_msgs::Odometry> {
  typedef nav_msgs::msg::Odometry type;
};
template <> struct Ros2MessageType<nav_msgs::Path> {
  typedef nav_msgs::msg::Path type;
};

template <typename Ros1ServiceT> struct Ros2ServiceType;
template <> struct Ros2ServiceType<nav_msgs::GetMap> {
  typedef nav_msgs::srv::GetMap type;
};


template <>
inline void convert_2_to_1<nav_msgs::MapMetaData, nav_msgs::msg::MapMetaData>(
    const nav_msgs::msg::MapMetaData& ros2_msg,
    nav_msgs::MapMetaData& ros1_msg) {
  convert_2_to_1(ros2_msg.map_load_time, ros1_msg.map_load_time);
  ros1_msg.resolution = ros2_msg.resolution;
  ros1_msg.width = ros2_msg.width;
  ros1_msg.height = ros2_msg.height;
  convert_2_to_1(ros2_msg.origin, ros1_msg.origin);
}

template <>
inline void convert_1_to_2<nav_msgs::msg::MapMetaData, nav_msgs::MapMetaData>(
    const nav_msgs::MapMetaData& ros1_msg,
    nav_msgs::msg::MapMetaData& ros2_msg) {
  convert_1_to_2(ros1_msg.map_load_time, ros2_msg.map_load_time);
  ros2_msg.resolution = ros1_msg.resolution;
  ros2_msg.width = ros1_msg.width;
  ros2_msg.height = ros1_msg.height;
  convert_1_to_2(ros1_msg.origin, ros2_msg.origin);
}

template <>
inline void convert_2_to_1<nav_msgs::OccupancyGrid, nav_msgs::msg::OccupancyGrid>(
    const nav_msgs::msg::OccupancyGrid& ros2_msg,
    nav_msgs::OccupancyGrid& ros1_msg) {
  convert_2_to_1(ros2_msg.header, ros1_msg.header);
  convert_2_to_1(ros2_msg.info, ros1_msg.info);
  convert_2_to_1(ros2_msg.data, ros1_msg.data);
}

template <>
inline void convert_2_to_1<nav_msgs::OccupancyGrid, nav_msgs::msg::OccupancyGrid>(
    nav_msgs::msg::OccupancyGrid&& ros2_msg,
    nav_msgs::OccupancyGrid& ros1_msg) {
  convert_2_to_1(std::move(ros2_msg.header), ros1_msg.header);
  convert_2_to_1(std::move(ros2_msg.info), ros1_msg.info);
  convert_2_to_1(std::move(ros2_msg.data), ros1_msg.data);
}

template <>
inline void convert_1_to_2<nav_msgs::msg::OccupancyGrid, nav_msgs::OccupancyGrid>(
    const nav_msgs::OccupancyGrid& ros1_msg,
    nav_msgs::msg::OccupancyGrid& ros2_msg) {
  convert_1_to_2(ros1_msg.header, ros2_msg.header);
  convert_1_to_2(ros1_msg.info, ros2_msg.info);
  convert_1_to_2(ros1_msg.data, ros2_msg.data);
}

template <>
inline void convert_1_to_2<nav_msgs::msg::OccupancyGrid, nav_msgs::OccupancyGrid>(
    nav_msgs::OccupancyGrid&& ros1_msg,
    nav_msgs::msg::OccupancyGrid& ros2_msg) {
  convert_1_to_2(std::move(ros1_msg.header), ros2_msg.header);
  convert_1_to_2(std::move(ros1_msg.info), ros2_msg.info);
  convert_1_to_2(std::move(ros1_msg.data), ros2_msg.data);
}

template <>
inline void convert_2_to_1<nav_msgs::Odometry, nav_msgs::msg::Odometry>(
    const nav_msgs::msg::Odometry& ros2_msg,
    nav_msgs::Odometry& ros1_msg) {
  convert_2_to_1(ros2_msg.header, ros1_msg.header);
  ros1_msg.child_frame_id = ros2_msg.child_frame_id;
  convert_2_to_1(ros2_msg.pose.pose.position, ros1_msg.pose.pose.position);
  convert_2_to_1(ros2_msg.pose.pose.orientation, ros1_msg.pose.pose.orientation);
  convert_2_to_1(ros2_msg.pose.covariance, ros1_msg.pose.covariance);
  convert_2_to_1(ros2_msg.twist.twist.linear, ros1_msg.twist.twist.linear);
  convert_2_to_1(ros2_msg.twist.twist.angular, ros1_msg.twist.twist.angular);
  convert_2_to_1(ros2_msg.twist.covariance, ros1_msg.twist.covariance);
}

template <>
inline void convert_1_to_2<nav_msgs::msg::Odometry, nav_msgs::Odometry>(
    const nav_msgs::Odometry& ros1_msg,
    nav_msgs::msg::Odometry& ros2_msg) {
  convert_1_to_2(ros1_msg.header, ros2_msg.header);
  ros2_msg.child_frame_id = ros1_msg.child_frame_id;
  convert_1_to_2(ros1_msg.pose.pose.position, ros2_msg.pose.pose.position);
  convert_1_to_2(ros1_msg.pose.pose.orientation, ros2_msg.pose.pose.orientation);
  convert_1_to_2(ros1_msg.pose.covariance, ros2_msg.pose.covariance);
  convert_1_to_2(ros1_msg.twist.twist.linear, ros2_msg.twist.twist.linear);
  convert_1_to_2(ros1_msg.twist.twist.angular, ros2_msg.twist.twist.angular);
  convert_1_to_2(ros1_msg.twist.covariance, ros2_msg.twist.covariance);
}

template <>
inline void convert_2_to_1<nav_msgs::Path, nav_msgs::msg::Path>(
    const nav_msgs::msg::Path& ros2_msg,
    nav_msgs::Path& ros1_msg) {
  convert_2_to_1(ros2_msg.header, ros1_msg.header);
  convert_2_to_1(ros2_msg.poses, ros1_msg.poses);
}

template <>
inline void convert_1_to_2<nav_msgs::msg::Path, nav_msgs::Path>(
    const nav_msgs::Path& ros1_msg,
    nav_msgs::msg::Path& ros2_msg) {
  convert_1_to_2(ros1_msg.header, ros2_msg.header);
  convert_1_to_2(ros1_msg.poses, ros2_msg.poses);
}

template <>
inline void convert_2_to_1<nav_msgs::GetMap::Request, nav_msgs::srv::GetMap::Request>(
    const nav_msgs::srv::GetMap::Request&,
    nav_msgs::GetMap::Request&) {
  // empty
}

template <>
inline void convert_1_to_2<nav_msgs::srv::GetMap::Response, nav_msgs::GetMap::Response>(
    const nav_msgs::GetMap::Response& ros1_msg,
    nav_msgs::srv::GetMap::Response& ros2_msg) {
  convert_1_to_2(ros1_msg.map, ros2_msg.map);
}

template <>
inline void convert_1_to_2<nav_msgs::srv::GetMap::Response, nav_msgs::GetMap::Response>(
    nav_msgs::GetMap::Response&& ros1_msg,
    nav_msgs::srv::GetMap::Response& ros2_msg) {
  convert_1_to_2(std::move(ros1_msg.map), ros2_msg.map);
}

}  // namespace conversions
}  // namespace ros2in1_support

#endif  // ROS2_SUPPORT
#endif  // ROS2IN1_SUPPORT_CONVERSIONS_NAV_MSGS_H
