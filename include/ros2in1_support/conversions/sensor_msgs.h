/******************************************************************************
 *  Copyright (c) 2024, Intermodalics BVBA                                    *
 *  All rights reserved.                                                      *
 ******************************************************************************/

#ifndef ROS2IN1_SUPPORT_CONVERSIONS_SENSOR_MSGS_H
#define ROS2IN1_SUPPORT_CONVERSIONS_SENSOR_MSGS_H
#ifdef ROS2_SUPPORT

#include <algorithm>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/PointCloud2.h>

#include "builtin_interfaces.h"
#include "geometry_msgs.h"

namespace ros2in1_support {
namespace conversions {

template <typename Ros1MessageT> struct Ros2MessageType;
template <> struct Ros2MessageType<sensor_msgs::Imu> {
  typedef sensor_msgs::msg::Imu type;
};
template <> struct Ros2MessageType<sensor_msgs::Image> {
  typedef sensor_msgs::msg::Image type;
};
template <> struct Ros2MessageType<sensor_msgs::CameraInfo> {
  typedef sensor_msgs::msg::CameraInfo type;
};
template <> struct Ros2MessageType<sensor_msgs::LaserScan> {
  typedef sensor_msgs::msg::LaserScan type;
};
template <> struct Ros2MessageType<sensor_msgs::NavSatFix> {
  typedef sensor_msgs::msg::NavSatFix type;
};
template <> struct Ros2MessageType<sensor_msgs::NavSatStatus> {
  typedef sensor_msgs::msg::NavSatStatus type;
};
template <> struct Ros2MessageType<sensor_msgs::PointCloud2> {
  typedef sensor_msgs::msg::PointCloud2 type;
};

template <>
inline void convert_2_to_1<sensor_msgs::Imu, sensor_msgs::msg::Imu>(
    const sensor_msgs::msg::Imu& ros2_msg,
    sensor_msgs::Imu& ros1_msg) {
  convert_2_to_1(ros2_msg.header.stamp, ros1_msg.header.stamp);
  ros1_msg.header.frame_id = ros2_msg.header.frame_id;
  convert_2_to_1(ros2_msg.orientation, ros1_msg.orientation);
  std::copy(ros2_msg.orientation_covariance.begin(), ros2_msg.orientation_covariance.end(), ros1_msg.orientation_covariance.begin());
  convert_2_to_1(ros2_msg.angular_velocity, ros1_msg.angular_velocity);
  std::copy(ros2_msg.angular_velocity_covariance.begin(), ros2_msg.angular_velocity_covariance.end(), ros1_msg.angular_velocity_covariance.begin());
  convert_2_to_1(ros2_msg.linear_acceleration, ros1_msg.linear_acceleration);
  std::copy(ros2_msg.linear_acceleration_covariance.begin(), ros2_msg.linear_acceleration_covariance.end(), ros1_msg.linear_acceleration_covariance.begin());
}

template <>
inline void convert_2_to_1<sensor_msgs::Image, sensor_msgs::msg::Image>(
    const sensor_msgs::msg::Image& ros2_msg,
    sensor_msgs::Image& ros1_msg) {
  convert_2_to_1(ros2_msg.header.stamp, ros1_msg.header.stamp);
  ros1_msg.header.frame_id = ros2_msg.header.frame_id;
  ros1_msg.height = ros2_msg.height;
  ros1_msg.width = ros2_msg.width;
  ros1_msg.encoding = ros2_msg.encoding;
  ros1_msg.is_bigendian = ros2_msg.is_bigendian;
  ros1_msg.step = ros2_msg.step;
  ros1_msg.data = ros2_msg.data;
}

template <>
inline void convert_2_to_1<sensor_msgs::CameraInfo, sensor_msgs::msg::CameraInfo>(
    const sensor_msgs::msg::CameraInfo& ros2_msg,
    sensor_msgs::CameraInfo& ros1_msg) {
  convert_2_to_1(ros2_msg.header.stamp, ros1_msg.header.stamp);
  ros1_msg.header.frame_id = ros2_msg.header.frame_id;
  ros1_msg.height = ros2_msg.height;
  ros1_msg.width = ros2_msg.width;
  ros1_msg.distortion_model = ros2_msg.distortion_model;
  ros1_msg.D = ros2_msg.d;
  std::copy(ros2_msg.k.begin(), ros2_msg.k.end(), ros1_msg.K.begin());
  std::copy(ros2_msg.r.begin(), ros2_msg.r.end(), ros1_msg.P.begin());
  std::copy(ros2_msg.p.begin(), ros2_msg.p.end(), ros1_msg.P.begin());
  ros1_msg.binning_x = ros2_msg.binning_x;
  ros1_msg.binning_y = ros2_msg.binning_y;
  ros1_msg.roi.x_offset = ros2_msg.roi.x_offset;
  ros1_msg.roi.y_offset = ros2_msg.roi.y_offset;
  ros1_msg.roi.height = ros2_msg.roi.height;
  ros1_msg.roi.width = ros2_msg.roi.width;
  ros1_msg.roi.do_rectify = ros2_msg.roi.do_rectify;
}

template <>
inline void convert_2_to_1<sensor_msgs::LaserScan, sensor_msgs::msg::LaserScan>(
    const sensor_msgs::msg::LaserScan& ros2_msg,
    sensor_msgs::LaserScan& ros1_msg) {
  convert_2_to_1(ros2_msg.header.stamp, ros1_msg.header.stamp);
  ros1_msg.header.frame_id = ros2_msg.header.frame_id;
  ros1_msg.header.frame_id = ros2_msg.header.frame_id;
  ros1_msg.angle_min = ros2_msg.angle_min;
  ros1_msg.angle_max = ros2_msg.angle_max;
  ros1_msg.angle_increment = ros2_msg.angle_increment;
  ros1_msg.time_increment = ros2_msg.time_increment;
  ros1_msg.scan_time = ros2_msg.scan_time;
  ros1_msg.range_min = ros2_msg.range_min;
  ros1_msg.range_max = ros2_msg.range_max;
  ros1_msg.ranges = ros2_msg.ranges;
  ros1_msg.intensities = ros2_msg.intensities;
}

template <>
inline void convert_2_to_1<sensor_msgs::NavSatStatus, sensor_msgs::msg::NavSatStatus>(
    const sensor_msgs::msg::NavSatStatus& ros2_msg,
    sensor_msgs::NavSatStatus& ros1_msg) {
  ros1_msg.status = ros2_msg.status;
  ros1_msg.service = ros2_msg.service;
}

template <>
inline void convert_1_to_2<sensor_msgs::msg::NavSatStatus, sensor_msgs::NavSatStatus>(
    const sensor_msgs::NavSatStatus& ros1_msg,
    sensor_msgs::msg::NavSatStatus& ros2_msg) {
  ros2_msg.status = ros1_msg.status;
  ros2_msg.service = ros1_msg.service;
}

template <>
inline void convert_2_to_1<sensor_msgs::NavSatFix, sensor_msgs::msg::NavSatFix>(
    const sensor_msgs::msg::NavSatFix& ros2_msg,
    sensor_msgs::NavSatFix& ros1_msg) {
  convert_2_to_1(ros2_msg.header.stamp, ros1_msg.header.stamp);
  convert_2_to_1(ros2_msg.status, ros1_msg.status);
  ros1_msg.latitude = ros2_msg.latitude;
  ros1_msg.longitude = ros2_msg.longitude;
  ros1_msg.altitude = ros2_msg.altitude;
  std::copy(ros2_msg.position_covariance.begin(), ros2_msg.position_covariance.end(), ros1_msg.position_covariance.begin());
  ros1_msg.position_covariance_type = ros2_msg.position_covariance_type;
}

template <>
inline void convert_1_to_2<sensor_msgs::msg::NavSatFix, sensor_msgs::NavSatFix>(
    const sensor_msgs::NavSatFix& ros1_msg,
    sensor_msgs::msg::NavSatFix& ros2_msg) {
  convert_1_to_2(ros1_msg.header.stamp, ros2_msg.header.stamp);
  convert_1_to_2(ros1_msg.status, ros2_msg.status);
  ros2_msg.latitude = ros1_msg.latitude;
  ros2_msg.longitude = ros1_msg.longitude;
  ros2_msg.altitude = ros1_msg.altitude;
  std::copy(ros1_msg.position_covariance.begin(), ros1_msg.position_covariance.end(), ros2_msg.position_covariance.begin());
  ros2_msg.position_covariance_type = ros1_msg.position_covariance_type;
}

template <>
inline void convert_2_to_1<sensor_msgs::PointCloud2, sensor_msgs::msg::PointCloud2>(
    const sensor_msgs::msg::PointCloud2& ros2_msg,
    sensor_msgs::PointCloud2& ros1_msg) {
  convert_2_to_1(ros2_msg.header.stamp, ros1_msg.header.stamp);
  ros1_msg.header.frame_id = ros2_msg.header.frame_id;
  ros1_msg.height = ros2_msg.height;
  ros1_msg.width = ros2_msg.width;
  ros1_msg.fields.reserve(ros2_msg.fields.size());
  for (const sensor_msgs::msg::PointField& ros2_field : ros2_msg.fields) {
    sensor_msgs::PointField ros1_field;
    ros1_field.name = ros2_field.name;
    ros1_field.offset = ros2_field.offset;
    ros1_field.datatype = ros2_field.datatype;
    ros1_field.count = ros2_field.count;
    ros1_msg.fields.emplace_back(std::move(ros1_field));
  }
  ros1_msg.is_bigendian = ros2_msg.is_bigendian;
  ros1_msg.point_step = ros2_msg.point_step;
  ros1_msg.row_step = ros2_msg.row_step;
  ros1_msg.data = ros2_msg.data;
  ros1_msg.is_dense = ros2_msg.is_dense;
}

}  // namespace conversions
}  // namespace ros2in1_support

#endif  // ROS2_SUPPORT
#endif  // ROS2IN1_SUPPORT_CONVERSIONS_SENSOR_MSGS_H
