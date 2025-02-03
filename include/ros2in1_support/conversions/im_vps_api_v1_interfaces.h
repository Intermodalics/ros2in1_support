/******************************************************************************
 *  Copyright (c) 2024, Intermodalics BVBA                                    *
 *  All rights reserved.                                                      *
 ******************************************************************************/

#ifndef ROS2IN1_SUPPORT_CONVERSIONS_IM_VPS_API_V1_INTERFACES_H
#define ROS2IN1_SUPPORT_CONVERSIONS_IM_VPS_API_V1_INTERFACES_H
#ifdef ROS2_SUPPORT

#include <algorithm>

#include <glog/logging.h>

#include <im_vps_api_v1_interfaces/msg/gnss_status.hpp>
#include <im_vps_api_v1_interfaces/msg/heartbeat2.hpp>
#include <im_vps_api_v1_interfaces/msg/header2.hpp>
#include <im_vps_api_v1_interfaces/msg/local_fix.hpp>
#include <im_vps_api_v1_interfaces/msg/localization.hpp>
#include <im_vps_api_v1_interfaces/msg/localization_status.hpp>
#include <im_vps_api_v1_interfaces/msg/localization_status_stamped.hpp>
#include <im_vps_api_v1_interfaces/msg/mapping_status.hpp>
#include <im_vps_api_v1_interfaces/msg/mapping_status_stamped.hpp>
#include <im_vps_api_v1_interfaces/msg/point3_d.hpp>
#include <im_vps_api_v1_interfaces/msg/wgs84_pose.hpp>
#include <im_vps_api_v1_interfaces/msg/wgs84_pose_stamped.hpp>
#include <im_vps_api_v1_interfaces/msg/wgs84_position.hpp>

#include <im_vps_api_v1_interfaces/GNSSStatus.h>
#include <im_vps_api_v1_interfaces/Heartbeat2.h>
#include <im_vps_api_v1_interfaces/Header2.h>
#include <im_vps_api_v1_interfaces/LocalFix.h>
#include <im_vps_api_v1_interfaces/Localization.h>
#include <im_vps_api_v1_interfaces/LocalizationStatus.h>
#include <im_vps_api_v1_interfaces/LocalizationStatusStamped.h>
#include <im_vps_api_v1_interfaces/MappingStatus.h>
#include <im_vps_api_v1_interfaces/MappingStatusStamped.h>
#include <im_vps_api_v1_interfaces/Point3D.h>
#include <im_vps_api_v1_interfaces/WGS84Pose.h>
#include <im_vps_api_v1_interfaces/WGS84PoseStamped.h>
#include <im_vps_api_v1_interfaces/WGS84Position.h>

#include <im_vps_api_v1_interfaces/srv/abort_mission.hpp>
#include <im_vps_api_v1_interfaces/srv/abort_mapping.hpp>
#include <im_vps_api_v1_interfaces/srv/erase_map.hpp>
#include <im_vps_api_v1_interfaces/srv/get_map_list.hpp>
#include <im_vps_api_v1_interfaces/srv/get_map_metadata.hpp>
#include <im_vps_api_v1_interfaces/srv/load_map.hpp>
#include <im_vps_api_v1_interfaces/srv/reset.hpp>
#include <im_vps_api_v1_interfaces/srv/start_mission.hpp>
#include <im_vps_api_v1_interfaces/srv/start_mapping.hpp>
#include <im_vps_api_v1_interfaces/srv/stop_mission.hpp>
#include <im_vps_api_v1_interfaces/srv/stop_mapping.hpp>
#include <im_vps_api_v1_interfaces/srv/set_map_metadata.hpp>
#include <im_vps_api_v1_interfaces/srv/unload_map.hpp>

#include <im_vps_api_v1_interfaces/AbortMission.h>
#include <im_vps_api_v1_interfaces/AbortMapping.h>
#include <im_vps_api_v1_interfaces/EraseMap.h>
#include <im_vps_api_v1_interfaces/GetMapList.h>
#include <im_vps_api_v1_interfaces/GetMapMetadata.h>
#include <im_vps_api_v1_interfaces/LoadMap.h>
#include <im_vps_api_v1_interfaces/Reset.h>
#include <im_vps_api_v1_interfaces/StartMission.h>
#include <im_vps_api_v1_interfaces/StartMapping.h>
#include <im_vps_api_v1_interfaces/StopMission.h>
#include <im_vps_api_v1_interfaces/StopMapping.h>
#include <im_vps_api_v1_interfaces/SetMapMetadata.h>
#include <im_vps_api_v1_interfaces/UnloadMap.h>

#include "common.h"
#include "builtin_interfaces.h"

namespace ros2in1_support {
namespace conversions {

template <typename Ros1MessageT> struct Ros2MessageType;
template <>struct Ros2MessageType<im_vps_api_v1_interfaces::GNSSStatus> {
  typedef im_vps_api_v1_interfaces::msg::GNSSStatus type;
};
template <> struct Ros2MessageType<im_vps_api_v1_interfaces::Header2> {
  typedef im_vps_api_v1_interfaces::msg::Header2 type;
};
template <> struct Ros2MessageType<im_vps_api_v1_interfaces::Heartbeat2> {
  typedef im_vps_api_v1_interfaces::msg::Heartbeat2 type;
};
template <> struct Ros2MessageType<im_vps_api_v1_interfaces::Localization> {
  typedef im_vps_api_v1_interfaces::msg::Localization type;
};
template <> struct Ros2MessageType<im_vps_api_v1_interfaces::LocalizationStatus> {
  typedef im_vps_api_v1_interfaces::msg::LocalizationStatus type;
};
template <> struct Ros2MessageType<im_vps_api_v1_interfaces::LocalizationStatusStamped> {
  typedef im_vps_api_v1_interfaces::msg::LocalizationStatusStamped type;
};
template <>struct Ros2MessageType<im_vps_api_v1_interfaces::LocalFix> {
  typedef im_vps_api_v1_interfaces::msg::LocalFix type;
};
template <> struct Ros2MessageType<im_vps_api_v1_interfaces::MappingStatus> {
  typedef im_vps_api_v1_interfaces::msg::MappingStatus type;
};
template <> struct Ros2MessageType<im_vps_api_v1_interfaces::MappingStatusStamped> {
  typedef im_vps_api_v1_interfaces::msg::MappingStatusStamped type;
};
template <>struct Ros2MessageType<im_vps_api_v1_interfaces::Point3D> {
  typedef im_vps_api_v1_interfaces::msg::Point3D type;
};
template <> struct Ros2MessageType<im_vps_api_v1_interfaces::RoverId> {
  typedef im_vps_api_v1_interfaces::msg::RoverId type;
};
template <> struct Ros2MessageType<im_vps_api_v1_interfaces::VioState> {
  typedef im_vps_api_v1_interfaces::msg::VioState type;
};
template <>struct Ros2MessageType<im_vps_api_v1_interfaces::WGS84Pose> {
  typedef im_vps_api_v1_interfaces::msg::WGS84Pose type;
};
template <>struct Ros2MessageType<im_vps_api_v1_interfaces::WGS84PoseStamped> {
  typedef im_vps_api_v1_interfaces::msg::WGS84PoseStamped type;
};
template <>struct Ros2MessageType<im_vps_api_v1_interfaces::WGS84Position> {
  typedef im_vps_api_v1_interfaces::msg::WGS84Position type;
};

template <typename Ros1ServiceT> struct Ros2ServiceType;
template <> struct Ros2ServiceType<im_vps_api_v1_interfaces::AbortMission> {
  typedef im_vps_api_v1_interfaces::srv::AbortMission type;
};
template <> struct Ros2ServiceType<im_vps_api_v1_interfaces::AbortMapping> {
  typedef im_vps_api_v1_interfaces::srv::AbortMapping type;
};
template <> struct Ros2ServiceType<im_vps_api_v1_interfaces::EraseMap> {
  typedef im_vps_api_v1_interfaces::srv::EraseMap type;
};
template <> struct Ros2ServiceType<im_vps_api_v1_interfaces::GetMapList> {
  typedef im_vps_api_v1_interfaces::srv::GetMapList type;
};
template <> struct Ros2ServiceType<im_vps_api_v1_interfaces::GetMapMetadata> {
  typedef im_vps_api_v1_interfaces::srv::GetMapMetadata type;
};
template <> struct Ros2ServiceType<im_vps_api_v1_interfaces::LoadMap> {
  typedef im_vps_api_v1_interfaces::srv::LoadMap type;
};
template <> struct Ros2ServiceType<im_vps_api_v1_interfaces::Reset> {
  typedef im_vps_api_v1_interfaces::srv::Reset type;
};
template <> struct Ros2ServiceType<im_vps_api_v1_interfaces::StartMission> {
  typedef im_vps_api_v1_interfaces::srv::StartMission type;
};
template <> struct Ros2ServiceType<im_vps_api_v1_interfaces::StartMapping> {
  typedef im_vps_api_v1_interfaces::srv::StartMapping type;
};
template <> struct Ros2ServiceType<im_vps_api_v1_interfaces::StopMission> {
  typedef im_vps_api_v1_interfaces::srv::StopMission type;
};
template <> struct Ros2ServiceType<im_vps_api_v1_interfaces::StopMapping> {
  typedef im_vps_api_v1_interfaces::srv::StopMapping type;
};
template <> struct Ros2ServiceType<im_vps_api_v1_interfaces::SetMapMetadata> {
  typedef im_vps_api_v1_interfaces::srv::SetMapMetadata type;
};
template <> struct Ros2ServiceType<im_vps_api_v1_interfaces::UnloadMap> {
  typedef im_vps_api_v1_interfaces::srv::UnloadMap type;
};

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::Header2, im_vps_api_v1_interfaces::msg::Header2>(
    const im_vps_api_v1_interfaces::msg::Header2& ros2_msg,
    im_vps_api_v1_interfaces::Header2& ros1_msg) {
  convert_2_to_1(ros2_msg.stamp, ros1_msg.stamp);
  ros1_msg.frame_id = ros2_msg.frame_id;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::Header2, im_vps_api_v1_interfaces::Header2>(
    const im_vps_api_v1_interfaces::Header2& ros1_msg,
    im_vps_api_v1_interfaces::msg::Header2& ros2_msg) {
  convert_1_to_2(ros1_msg.stamp, ros2_msg.stamp);
  ros2_msg.frame_id = ros1_msg.frame_id;
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::Point3D, im_vps_api_v1_interfaces::msg::Point3D>(
    const im_vps_api_v1_interfaces::msg::Point3D& ros2_msg,
    im_vps_api_v1_interfaces::Point3D& ros1_msg) {
  ros1_msg.x = ros2_msg.x;
  ros1_msg.y = ros2_msg.y;
  ros1_msg.z = ros2_msg.z;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::Point3D, im_vps_api_v1_interfaces::Point3D>(
    const im_vps_api_v1_interfaces::Point3D& ros1_msg,
    im_vps_api_v1_interfaces::msg::Point3D& ros2_msg) {
  ros2_msg.x = ros1_msg.x;
  ros2_msg.y = ros1_msg.y;
  ros2_msg.z = ros1_msg.z;
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::Quaternion, im_vps_api_v1_interfaces::msg::Quaternion>(
    const im_vps_api_v1_interfaces::msg::Quaternion& ros2_msg,
    im_vps_api_v1_interfaces::Quaternion& ros1_msg) {
  ros1_msg.x = ros2_msg.x;
  ros1_msg.y = ros2_msg.y;
  ros1_msg.z = ros2_msg.z;
  ros1_msg.w = ros2_msg.w;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::Quaternion, im_vps_api_v1_interfaces::Quaternion>(
    const im_vps_api_v1_interfaces::Quaternion& ros1_msg,
    im_vps_api_v1_interfaces::msg::Quaternion& ros2_msg) {
  ros2_msg.x = ros1_msg.x;
  ros2_msg.y = ros1_msg.y;
  ros2_msg.z = ros1_msg.z;
  ros2_msg.w = ros1_msg.w;
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::Vector3D, im_vps_api_v1_interfaces::msg::Vector3D>(
    const im_vps_api_v1_interfaces::msg::Vector3D& ros2_msg,
    im_vps_api_v1_interfaces::Vector3D& ros1_msg) {
  ros1_msg.x = ros2_msg.x;
  ros1_msg.y = ros2_msg.y;
  ros1_msg.z = ros2_msg.z;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::Vector3D, im_vps_api_v1_interfaces::Vector3D>(
    const im_vps_api_v1_interfaces::Vector3D& ros1_msg,
    im_vps_api_v1_interfaces::msg::Vector3D& ros2_msg) {
  ros2_msg.x = ros1_msg.x;
  ros2_msg.y = ros1_msg.y;
  ros2_msg.z = ros1_msg.z;
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::Pose3D, im_vps_api_v1_interfaces::msg::Pose3D>(
    const im_vps_api_v1_interfaces::msg::Pose3D& ros2_msg,
    im_vps_api_v1_interfaces::Pose3D& ros1_msg) {
  convert_2_to_1(ros2_msg.position, ros1_msg.position);
  convert_2_to_1(ros2_msg.orientation, ros1_msg.orientation);
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::Pose3D, im_vps_api_v1_interfaces::Pose3D>(
    const im_vps_api_v1_interfaces::Pose3D& ros1_msg,
    im_vps_api_v1_interfaces::msg::Pose3D& ros2_msg) {
  convert_1_to_2(ros1_msg.position, ros2_msg.position);
  convert_1_to_2(ros1_msg.orientation, ros2_msg.orientation);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::Twist3D, im_vps_api_v1_interfaces::msg::Twist3D>(
    const im_vps_api_v1_interfaces::msg::Twist3D& ros2_msg,
    im_vps_api_v1_interfaces::Twist3D& ros1_msg) {
  convert_2_to_1(ros2_msg.linear, ros1_msg.linear);
  convert_2_to_1(ros2_msg.angular, ros1_msg.angular);
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::Twist3D, im_vps_api_v1_interfaces::Twist3D>(
    const im_vps_api_v1_interfaces::Twist3D& ros1_msg,
    im_vps_api_v1_interfaces::msg::Twist3D& ros2_msg) {
  convert_1_to_2(ros1_msg.linear, ros2_msg.linear);
  convert_1_to_2(ros1_msg.angular, ros2_msg.angular);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::LocalizationStatus, im_vps_api_v1_interfaces::msg::LocalizationStatus>(
    const im_vps_api_v1_interfaces::msg::LocalizationStatus& ros2_msg,
    im_vps_api_v1_interfaces::LocalizationStatus& ros1_msg) {
  (void)ros1_msg;
  (void)ros2_msg;
  LOG(FATAL) << "not implemented";
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::LocalizationStatus, im_vps_api_v1_interfaces::LocalizationStatus>(
    const im_vps_api_v1_interfaces::LocalizationStatus& ros1_msg,
    im_vps_api_v1_interfaces::msg::LocalizationStatus& ros2_msg) {
  ros2_msg.loaded_map_uuid = ros1_msg.loaded_map_uuid;
  ros2_msg.status = ros1_msg.status;
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::LocalizationStatusStamped, im_vps_api_v1_interfaces::msg::LocalizationStatusStamped>(
    const im_vps_api_v1_interfaces::msg::LocalizationStatusStamped& ros2_msg,
    im_vps_api_v1_interfaces::LocalizationStatusStamped& ros1_msg) {
  (void)ros1_msg;
  (void)ros2_msg;
  LOG(FATAL) << "not implemented";
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::LocalizationStatusStamped, im_vps_api_v1_interfaces::LocalizationStatusStamped>(
    const im_vps_api_v1_interfaces::LocalizationStatusStamped& ros1_msg,
    im_vps_api_v1_interfaces::msg::LocalizationStatusStamped& ros2_msg) {
  convert_1_to_2(ros1_msg.stamp, ros2_msg.stamp);
  convert_1_to_2(ros1_msg.status, ros2_msg.status);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::MappingStatus, im_vps_api_v1_interfaces::msg::MappingStatus>(
    const im_vps_api_v1_interfaces::msg::MappingStatus& ros2_msg,
    im_vps_api_v1_interfaces::MappingStatus& ros1_msg) {
  (void)ros1_msg;
  (void)ros2_msg;
  LOG(FATAL) << "not implemented";
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::MappingStatus, im_vps_api_v1_interfaces::MappingStatus>(
    const im_vps_api_v1_interfaces::MappingStatus& ros1_msg,
    im_vps_api_v1_interfaces::msg::MappingStatus& ros2_msg) {
  ros2_msg.map_uuid = ros1_msg.map_uuid;
  ros2_msg.status = ros1_msg.status;
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::MappingStatusStamped, im_vps_api_v1_interfaces::msg::MappingStatusStamped>(
    const im_vps_api_v1_interfaces::msg::MappingStatusStamped& ros2_msg,
    im_vps_api_v1_interfaces::MappingStatusStamped& ros1_msg) {
  (void)ros1_msg;
  (void)ros2_msg;
  LOG(FATAL) << "not implemented";
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::MappingStatusStamped, im_vps_api_v1_interfaces::MappingStatusStamped>(
    const im_vps_api_v1_interfaces::MappingStatusStamped& ros1_msg,
    im_vps_api_v1_interfaces::msg::MappingStatusStamped& ros2_msg) {
  convert_1_to_2(ros1_msg.stamp, ros2_msg.stamp);
  convert_1_to_2(ros1_msg.status, ros2_msg.status);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::VioState, im_vps_api_v1_interfaces::msg::VioState>(
    const im_vps_api_v1_interfaces::msg::VioState& ros2_msg,
    im_vps_api_v1_interfaces::VioState& ros1_msg) {
  (void)ros1_msg;
  (void)ros2_msg;
  LOG(FATAL) << "not implemented";
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::VioState, im_vps_api_v1_interfaces::VioState>(
    const im_vps_api_v1_interfaces::VioState& ros1_msg,
    im_vps_api_v1_interfaces::msg::VioState& ros2_msg) {
  convert_1_to_2(ros1_msg.stamp, ros2_msg.stamp);
  ros2_msg.state = ros1_msg.state;
  ros2_msg.time_since_last_motion_s = ros1_msg.time_since_last_motion_s;
  convert_1_to_2(ros1_msg.gyro_bias, ros2_msg.gyro_bias);
  convert_1_to_2(ros1_msg.accel_bias, ros2_msg.accel_bias);
  convert_1_to_2(ros1_msg.wheel_odometry_bias, ros2_msg.wheel_odometry_bias);
  ros2_msg.max_num_features = ros1_msg.max_num_features;
  ros2_msg.num_tracked_features_per_camera = ros1_msg.num_tracked_features_per_camera;
  ros2_msg.num_tracked_stereo_features = ros1_msg.num_tracked_stereo_features;
  ros2_msg.feature_disparity_per_camera = ros1_msg.feature_disparity_per_camera;
  ros2_msg.feature_health = ros1_msg.feature_health;
  ros2_msg.filter_version = ros1_msg.filter_version;
  ros2_msg.filter_state = ros1_msg.filter_state;
  ros2_msg.filter_covariance = ros1_msg.filter_covariance;
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::Localization, im_vps_api_v1_interfaces::msg::Localization>(
    const im_vps_api_v1_interfaces::msg::Localization& ros2_msg,
    im_vps_api_v1_interfaces::Localization& ros1_msg) {
  (void)ros1_msg;
  (void)ros2_msg;
  LOG(FATAL) << "not implemented";
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::Localization, im_vps_api_v1_interfaces::Localization>(
    const im_vps_api_v1_interfaces::Localization& ros1_msg,
    im_vps_api_v1_interfaces::msg::Localization& ros2_msg) {
  convert_1_to_2(ros1_msg.header, ros2_msg.header);
  ros2_msg.child_frame_id = ros1_msg.child_frame_id;

  convert_1_to_2(ros1_msg.map_pose, ros2_msg.map_pose);
  std::copy(ros1_msg.map_pose_covariance.begin(), ros1_msg.map_pose_covariance.end(), ros2_msg.map_pose_covariance.begin());
  convert_1_to_2(ros1_msg.mission_pose, ros2_msg.mission_pose);
  std::copy(ros1_msg.mission_pose_covariance.begin(), ros1_msg.mission_pose_covariance.end(), ros2_msg.mission_pose_covariance.begin());
  convert_1_to_2(ros1_msg.twist, ros2_msg.twist);
  std::copy(ros1_msg.twist_covariance.begin(), ros1_msg.twist_covariance.end(), ros2_msg.twist_covariance.begin());

  convert_1_to_2(ros1_msg.localization_status, ros2_msg.localization_status);
  convert_1_to_2(ros1_msg.mapping_status, ros2_msg.mapping_status);
  convert_1_to_2(ros1_msg.vio, ros2_msg.vio);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::RoverId, im_vps_api_v1_interfaces::msg::RoverId>(
    const im_vps_api_v1_interfaces::msg::RoverId& ros2_msg,
    im_vps_api_v1_interfaces::RoverId& ros1_msg) {
  (void)ros1_msg;
  (void)ros2_msg;
  LOG(FATAL) << "not implemented";
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::RoverId, im_vps_api_v1_interfaces::RoverId>(
    const im_vps_api_v1_interfaces::RoverId& ros1_msg,
    im_vps_api_v1_interfaces::msg::RoverId& ros2_msg) {
  ros2_msg.id = ros1_msg.id;
  ros2_msg.custom_id = ros1_msg.custom_id;
  ros2_msg.mac_address = ros1_msg.mac_address;
  ros2_msg.ip_address = ros1_msg.ip_address;
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::Heartbeat2, im_vps_api_v1_interfaces::msg::Heartbeat2>(
    const im_vps_api_v1_interfaces::msg::Heartbeat2& ros2_msg,
    im_vps_api_v1_interfaces::Heartbeat2& ros1_msg) {
  (void)ros1_msg;
  (void)ros2_msg;
  LOG(FATAL) << "not implemented";
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::Heartbeat2, im_vps_api_v1_interfaces::Heartbeat2>(
    const im_vps_api_v1_interfaces::Heartbeat2& ros1_msg,
    im_vps_api_v1_interfaces::msg::Heartbeat2& ros2_msg) {
  convert_1_to_2(ros1_msg.header, ros2_msg.header);
  convert_1_to_2(ros1_msg.rover_id, ros2_msg.rover_id);
  convert_1_to_2(ros1_msg.pose, ros2_msg.pose);
  convert_1_to_2(ros1_msg.twist, ros2_msg.twist);
  convert_1_to_2(ros1_msg.localization_status, ros2_msg.localization_status);
  convert_1_to_2(ros1_msg.mapping_status, ros2_msg.mapping_status);
  ros2_msg.extra_data_format = ros1_msg.extra_data_format;
  ros2_msg.extra_data = ros1_msg.extra_data;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::ErrorType, im_vps_api_v1_interfaces::ErrorType>(
    im_vps_api_v1_interfaces::ErrorType&& ros1_msg,
    im_vps_api_v1_interfaces::msg::ErrorType& ros2_msg) {
  ros2_msg.code = ros1_msg.code;
  ros2_msg.msg = std::move(ros1_msg.msg);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::KeyValue, im_vps_api_v1_interfaces::msg::KeyValue>(
    const im_vps_api_v1_interfaces::msg::KeyValue& ros2_msg,
    im_vps_api_v1_interfaces::KeyValue& ros1_msg) {
  ros1_msg.key = ros2_msg.key;
  ros1_msg.value = ros2_msg.value;
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::KeyValue, im_vps_api_v1_interfaces::msg::KeyValue>(
    im_vps_api_v1_interfaces::msg::KeyValue&& ros2_msg,
    im_vps_api_v1_interfaces::KeyValue& ros1_msg) {
  ros1_msg.key = std::move(ros2_msg.key);
  ros1_msg.value = std::move(ros2_msg.value);
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::KeyValue, im_vps_api_v1_interfaces::KeyValue>(
    const im_vps_api_v1_interfaces::KeyValue& ros1_msg,
    im_vps_api_v1_interfaces::msg::KeyValue& ros2_msg) {
  ros2_msg.key = ros1_msg.key;
  ros2_msg.value = ros1_msg.value;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::KeyValue, im_vps_api_v1_interfaces::KeyValue>(
    im_vps_api_v1_interfaces::KeyValue&& ros1_msg,
    im_vps_api_v1_interfaces::msg::KeyValue& ros2_msg) {
  ros2_msg.key = std::move(ros1_msg.key);
  ros2_msg.value = std::move(ros1_msg.value);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::Configuration, im_vps_api_v1_interfaces::msg::Configuration>(
    const im_vps_api_v1_interfaces::msg::Configuration& ros2_msg,
    im_vps_api_v1_interfaces::Configuration& ros1_msg) {
  convert_2_to_1(ros2_msg.config, ros1_msg.config);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::Configuration, im_vps_api_v1_interfaces::msg::Configuration>(
    im_vps_api_v1_interfaces::msg::Configuration&& ros2_msg,
    im_vps_api_v1_interfaces::Configuration& ros1_msg) {
  convert_2_to_1(std::move(ros2_msg.config), ros1_msg.config);
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::Configuration, im_vps_api_v1_interfaces::Configuration>(
    const im_vps_api_v1_interfaces::Configuration& ros1_msg,
    im_vps_api_v1_interfaces::msg::Configuration& ros2_msg) {
  convert_1_to_2(ros1_msg.config, ros2_msg.config);
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::Configuration, im_vps_api_v1_interfaces::Configuration>(
    im_vps_api_v1_interfaces::Configuration&& ros1_msg,
    im_vps_api_v1_interfaces::msg::Configuration& ros2_msg) {
  convert_1_to_2(std::move(ros1_msg.config), ros2_msg.config);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::Uuid, im_vps_api_v1_interfaces::msg::Uuid>(
    const im_vps_api_v1_interfaces::msg::Uuid& ros2_msg,
    im_vps_api_v1_interfaces::Uuid& ros1_msg) {
  ros1_msg.data = ros2_msg.data;
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::Uuid, im_vps_api_v1_interfaces::msg::Uuid>(
    im_vps_api_v1_interfaces::msg::Uuid&& ros2_msg,
    im_vps_api_v1_interfaces::Uuid& ros1_msg) {
  ros1_msg.data = std::move(ros2_msg.data);
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::Uuid, im_vps_api_v1_interfaces::Uuid>(
    const im_vps_api_v1_interfaces::Uuid& ros1_msg,
    im_vps_api_v1_interfaces::msg::Uuid& ros2_msg) {
  ros2_msg.data = ros1_msg.data;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::Uuid, im_vps_api_v1_interfaces::Uuid>(
    im_vps_api_v1_interfaces::Uuid&& ros1_msg,
    im_vps_api_v1_interfaces::msg::Uuid& ros2_msg) {
  ros2_msg.data = std::move(ros1_msg.data);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::MapMetadata, im_vps_api_v1_interfaces::msg::MapMetadata>(
    const im_vps_api_v1_interfaces::msg::MapMetadata& ros2_msg,
    im_vps_api_v1_interfaces::MapMetadata& ros1_msg) {
  ros1_msg.timestamp = ros2_msg.timestamp;
  convert_2_to_1(ros2_msg.uuid, ros1_msg.uuid);
  convert_2_to_1(ros2_msg.extra_data, ros1_msg.extra_data);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::MapMetadata, im_vps_api_v1_interfaces::msg::MapMetadata>(
    im_vps_api_v1_interfaces::msg::MapMetadata&& ros2_msg,
    im_vps_api_v1_interfaces::MapMetadata& ros1_msg) {
  ros1_msg.timestamp = ros2_msg.timestamp;
  convert_2_to_1(std::move(ros2_msg.uuid), ros1_msg.uuid);
  convert_2_to_1(std::move(ros2_msg.extra_data), ros1_msg.extra_data);
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::MapMetadata, im_vps_api_v1_interfaces::MapMetadata>(
    const im_vps_api_v1_interfaces::MapMetadata& ros1_msg,
    im_vps_api_v1_interfaces::msg::MapMetadata& ros2_msg) {
  ros2_msg.timestamp = ros1_msg.timestamp;
  convert_1_to_2(ros1_msg.uuid, ros2_msg.uuid);
  convert_1_to_2(ros1_msg.extra_data, ros2_msg.extra_data);
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::MapMetadata, im_vps_api_v1_interfaces::MapMetadata>(
    im_vps_api_v1_interfaces::MapMetadata&& ros1_msg,
    im_vps_api_v1_interfaces::msg::MapMetadata& ros2_msg) {
  ros2_msg.timestamp = ros1_msg.timestamp;
  convert_1_to_2(std::move(ros1_msg.uuid), ros2_msg.uuid);
  convert_1_to_2(std::move(ros1_msg.extra_data), ros2_msg.extra_data);
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::GNSSStatus, im_vps_api_v1_interfaces::GNSSStatus>(
    const im_vps_api_v1_interfaces::GNSSStatus& ros1_msg,
    im_vps_api_v1_interfaces::msg::GNSSStatus& ros2_msg) {
  convert_1_to_2(ros1_msg.header, ros2_msg.header);
  ros2_msg.satellites_used = ros1_msg.satellites_used;
  std::copy(ros1_msg.satellite_used_prn.begin(), ros1_msg.satellite_used_prn.end(), ros2_msg.satellite_used_prn.begin());
  std::copy(ros1_msg.satellite_visible_z.begin(), ros1_msg.satellite_visible_z.end(), ros2_msg.satellite_visible_z.begin());
  std::copy(ros1_msg.satellite_visible_azimuth.begin(), ros1_msg.satellite_visible_azimuth.end(), ros2_msg.satellite_visible_azimuth.begin());
  std::copy(ros1_msg.satellite_visible_snr.begin(), ros1_msg.satellite_visible_snr.end(), ros2_msg.satellite_visible_snr.begin());
  ros2_msg.status = ros1_msg.status;
  ros2_msg.motion_source = ros1_msg.motion_source;
  ros2_msg.orientation_source = ros1_msg.orientation_source;
  ros2_msg.position_source = ros1_msg.position_source;
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::GNSSStatus, im_vps_api_v1_interfaces::msg::GNSSStatus>(
    const im_vps_api_v1_interfaces::msg::GNSSStatus& ros2_msg,
    im_vps_api_v1_interfaces::GNSSStatus& ros1_msg) {
  convert_2_to_1(ros2_msg.header, ros1_msg.header);
  ros1_msg.satellites_used = ros2_msg.satellites_used;
  std::copy(ros2_msg.satellite_used_prn.begin(), ros2_msg.satellite_used_prn.end(), ros1_msg.satellite_used_prn.begin());
  std::copy(ros2_msg.satellite_visible_z.begin(), ros2_msg.satellite_visible_z.end(), ros1_msg.satellite_visible_z.begin());
  std::copy(ros2_msg.satellite_visible_azimuth.begin(), ros2_msg.satellite_visible_azimuth.end(), ros1_msg.satellite_visible_azimuth.begin());
  std::copy(ros2_msg.satellite_visible_snr.begin(), ros2_msg.satellite_visible_snr.end(), ros1_msg.satellite_visible_snr.begin());
  ros1_msg.status = ros2_msg.status;
  ros1_msg.motion_source = ros2_msg.motion_source;
  ros1_msg.orientation_source = ros2_msg.orientation_source;
  ros1_msg.position_source = ros2_msg.position_source;
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::LocalFix, im_vps_api_v1_interfaces::msg::LocalFix>(
    const im_vps_api_v1_interfaces::msg::LocalFix& ros2_msg,
    im_vps_api_v1_interfaces::LocalFix& ros1_msg) {
  convert_2_to_1(ros2_msg.header, ros1_msg.header);
  convert_2_to_1(ros2_msg.status, ros1_msg.status);
  convert_2_to_1(ros2_msg.position, ros1_msg.position);
  ros1_msg.track = ros2_msg.track;
  ros1_msg.speed = ros2_msg.speed;
  ros1_msg.climb = ros2_msg.climb;
  ros1_msg.pitch = ros2_msg.pitch;
  ros1_msg.roll = ros2_msg.roll;
  ros1_msg.dip = ros2_msg.dip;
  ros1_msg.time = ros2_msg.time;
  ros1_msg.gdop = ros2_msg.gdop;
  ros1_msg.pdop = ros2_msg.pdop;
  ros1_msg.hdop = ros2_msg.hdop;
  ros1_msg.vdop = ros2_msg.vdop;
  ros1_msg.tdop = ros2_msg.tdop;
  ros1_msg.err = ros2_msg.err;
  ros1_msg.err_horz = ros2_msg.err_horz;
  ros1_msg.err_vert = ros2_msg.err_vert;
  ros1_msg.err_track = ros2_msg.err_track;
  ros1_msg.err_speed = ros2_msg.err_speed;
  ros1_msg.err_climb = ros2_msg.err_climb;
  ros1_msg.err_time = ros2_msg.err_time;
  ros1_msg.err_pitch = ros2_msg.err_pitch;
  ros1_msg.err_roll = ros2_msg.err_roll;
  ros1_msg.err_dip = ros2_msg.err_dip;
  ros1_msg.err_pitch = ros2_msg.err_pitch;
  std::copy(ros2_msg.position_covariance.begin(), ros2_msg.position_covariance.end(), ros1_msg.position_covariance.begin());
  ros1_msg.position_covariance_type = ros2_msg.position_covariance_type;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::LocalFix, im_vps_api_v1_interfaces::LocalFix>(
    const im_vps_api_v1_interfaces::LocalFix& ros1_msg,
    im_vps_api_v1_interfaces::msg::LocalFix& ros2_msg) {
  convert_1_to_2(ros1_msg.header, ros2_msg.header);
  convert_1_to_2(ros1_msg.header, ros2_msg.header);
  convert_1_to_2(ros1_msg.status, ros2_msg.status);
  convert_1_to_2(ros1_msg.position, ros2_msg.position);
  ros2_msg.track = ros1_msg.track;
  ros2_msg.speed = ros1_msg.speed;
  ros2_msg.climb = ros1_msg.climb;
  ros2_msg.pitch = ros1_msg.pitch;
  ros2_msg.roll = ros1_msg.roll;
  ros2_msg.dip = ros1_msg.dip;
  ros2_msg.time = ros1_msg.time;
  ros2_msg.gdop = ros1_msg.gdop;
  ros2_msg.pdop = ros1_msg.pdop;
  ros2_msg.hdop = ros1_msg.hdop;
  ros2_msg.vdop = ros1_msg.vdop;
  ros2_msg.tdop = ros1_msg.tdop;
  ros2_msg.err = ros1_msg.err;
  ros2_msg.err_horz = ros1_msg.err_horz;
  ros2_msg.err_vert = ros1_msg.err_vert;
  ros2_msg.err_track = ros1_msg.err_track;
  ros2_msg.err_speed = ros1_msg.err_speed;
  ros2_msg.err_climb = ros1_msg.err_climb;
  ros2_msg.err_time = ros1_msg.err_time;
  ros2_msg.err_pitch = ros1_msg.err_pitch;
  ros2_msg.err_roll = ros1_msg.err_roll;
  ros2_msg.err_dip = ros1_msg.err_dip;
  ros2_msg.err_pitch = ros1_msg.err_pitch;
  std::copy(ros1_msg.position_covariance.begin(), ros1_msg.position_covariance.end(), ros2_msg.position_covariance.begin());
  ros2_msg.position_covariance_type = ros1_msg.position_covariance_type;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::WGS84Position, im_vps_api_v1_interfaces::WGS84Position>(
    const im_vps_api_v1_interfaces::WGS84Position& ros1_msg,
    im_vps_api_v1_interfaces::msg::WGS84Position& ros2_msg) {
  ros2_msg.latitude = ros1_msg.latitude;
  ros2_msg.longitude = ros1_msg.longitude;
  ros2_msg.altitude = ros1_msg.altitude;
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::WGS84Position, im_vps_api_v1_interfaces::msg::WGS84Position>(
    const im_vps_api_v1_interfaces::msg::WGS84Position& ros2_msg,
    im_vps_api_v1_interfaces::WGS84Position& ros1_msg) {
  ros1_msg.latitude = ros2_msg.latitude;
  ros1_msg.longitude = ros2_msg.longitude;
  ros1_msg.altitude = ros2_msg.altitude;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::WGS84Pose, im_vps_api_v1_interfaces::WGS84Pose>(
    const im_vps_api_v1_interfaces::WGS84Pose& ros1_msg,
    im_vps_api_v1_interfaces::msg::WGS84Pose& ros2_msg) {
  convert_1_to_2(ros1_msg.position, ros2_msg.position);
  convert_1_to_2(ros1_msg.orientation_enu, ros2_msg.orientation_enu);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::WGS84Pose, im_vps_api_v1_interfaces::msg::WGS84Pose>(
    const im_vps_api_v1_interfaces::msg::WGS84Pose& ros2_msg,
    im_vps_api_v1_interfaces::WGS84Pose& ros1_msg) {
  convert_2_to_1(ros2_msg.position, ros1_msg.position);
  convert_2_to_1(ros2_msg.orientation_enu, ros1_msg.orientation_enu);
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::msg::WGS84PoseStamped, im_vps_api_v1_interfaces::WGS84PoseStamped>(
    const im_vps_api_v1_interfaces::WGS84PoseStamped& ros1_msg,
    im_vps_api_v1_interfaces::msg::WGS84PoseStamped& ros2_msg) {
  convert_1_to_2(ros1_msg.header, ros2_msg.header);
  convert_1_to_2(ros1_msg.pose, ros2_msg.pose);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::WGS84PoseStamped, im_vps_api_v1_interfaces::msg::WGS84PoseStamped>(
    const im_vps_api_v1_interfaces::msg::WGS84PoseStamped& ros2_msg,
    im_vps_api_v1_interfaces::WGS84PoseStamped& ros1_msg) {
  convert_2_to_1(ros2_msg.header, ros1_msg.header);
  convert_2_to_1(ros2_msg.pose, ros1_msg.pose);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::AbortMission::Request, im_vps_api_v1_interfaces::srv::AbortMission::Request>(
    const im_vps_api_v1_interfaces::srv::AbortMission::Request& ros2_request,
    im_vps_api_v1_interfaces::AbortMission::Request& ros1_request) {
  ros1_request.rover_id = ros2_request.rover_id;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::srv::AbortMission::Response, im_vps_api_v1_interfaces::AbortMission::Response>(
    im_vps_api_v1_interfaces::AbortMission::Response&& ros1_response,
    im_vps_api_v1_interfaces::srv::AbortMission::Response& ros2_response) {
  ros2_response.mission_uuid = ros1_response.mission_uuid;
  convert_1_to_2(std::move(ros1_response.result), ros2_response.result);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::AbortMapping::Request, im_vps_api_v1_interfaces::srv::AbortMapping::Request>(
    const im_vps_api_v1_interfaces::srv::AbortMapping::Request& ros2_request,
    im_vps_api_v1_interfaces::AbortMapping::Request& ros1_request) {
  ros1_request.rover_id = ros2_request.rover_id;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::srv::AbortMapping::Response, im_vps_api_v1_interfaces::AbortMapping::Response>(
    im_vps_api_v1_interfaces::AbortMapping::Response&& ros1_response,
    im_vps_api_v1_interfaces::srv::AbortMapping::Response& ros2_response) {
  ros2_response.map_uuid = std::move(ros1_response.map_uuid);
  convert_1_to_2(std::move(ros1_response.result), ros2_response.result);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::EraseMap::Request, im_vps_api_v1_interfaces::srv::EraseMap::Request>(
    const im_vps_api_v1_interfaces::srv::EraseMap::Request& ros2_request,
    im_vps_api_v1_interfaces::EraseMap::Request& ros1_request) {
  ros1_request.rover_id = ros2_request.rover_id;
  ros1_request.map_uuid = ros2_request.map_uuid;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::srv::EraseMap::Response, im_vps_api_v1_interfaces::EraseMap::Response>(
    im_vps_api_v1_interfaces::EraseMap::Response&& ros1_response,
    im_vps_api_v1_interfaces::srv::EraseMap::Response& ros2_response) {
  convert_1_to_2(std::move(ros1_response.result), ros2_response.result);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::GetMapList::Request, im_vps_api_v1_interfaces::srv::GetMapList::Request>(
    const im_vps_api_v1_interfaces::srv::GetMapList::Request& ros2_request,
    im_vps_api_v1_interfaces::GetMapList::Request& ros1_request) {
  ros1_request.rover_id = ros2_request.rover_id;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::srv::GetMapList::Response, im_vps_api_v1_interfaces::GetMapList::Response>(
    im_vps_api_v1_interfaces::GetMapList::Response&& ros1_response,
    im_vps_api_v1_interfaces::srv::GetMapList::Response& ros2_response) {
  ros2_response.maps_uuid = std::move(ros1_response.maps_uuid);
  convert_1_to_2(std::move(ros1_response.result), ros2_response.result);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::GetMapMetadata::Request, im_vps_api_v1_interfaces::srv::GetMapMetadata::Request>(
    const im_vps_api_v1_interfaces::srv::GetMapMetadata::Request& ros2_request,
    im_vps_api_v1_interfaces::GetMapMetadata::Request& ros1_request) {
  ros1_request.rover_id = ros2_request.rover_id;
  ros1_request.map_uuid = ros2_request.map_uuid;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::srv::GetMapMetadata::Response, im_vps_api_v1_interfaces::GetMapMetadata::Response>(
    im_vps_api_v1_interfaces::GetMapMetadata::Response&& ros1_response,
    im_vps_api_v1_interfaces::srv::GetMapMetadata::Response& ros2_response) {
  convert_1_to_2(std::move(ros1_response.metadata), ros2_response.metadata);
  convert_1_to_2(std::move(ros1_response.result), ros2_response.result);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::LoadMap::Request, im_vps_api_v1_interfaces::srv::LoadMap::Request>(
    const im_vps_api_v1_interfaces::srv::LoadMap::Request& ros2_request,
    im_vps_api_v1_interfaces::LoadMap::Request& ros1_request) {
  ros1_request.rover_id = ros2_request.rover_id;
  ros1_request.map_uuid = ros2_request.map_uuid;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::srv::LoadMap::Response, im_vps_api_v1_interfaces::LoadMap::Response>(
    im_vps_api_v1_interfaces::LoadMap::Response&& ros1_response,
    im_vps_api_v1_interfaces::srv::LoadMap::Response& ros2_response) {
  convert_1_to_2(std::move(ros1_response.result), ros2_response.result);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::Reset::Request, im_vps_api_v1_interfaces::srv::Reset::Request>(
    const im_vps_api_v1_interfaces::srv::Reset::Request& ros2_request,
    im_vps_api_v1_interfaces::Reset::Request& ros1_request) {
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::srv::Reset::Response, im_vps_api_v1_interfaces::Reset::Response>(
    im_vps_api_v1_interfaces::Reset::Response&& ros1_response,
    im_vps_api_v1_interfaces::srv::Reset::Response& ros2_response) {
  convert_1_to_2(std::move(ros1_response.result), ros2_response.result);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::StartMission::Request, im_vps_api_v1_interfaces::srv::StartMission::Request>(
    const im_vps_api_v1_interfaces::srv::StartMission::Request& ros2_request,
    im_vps_api_v1_interfaces::StartMission::Request& ros1_request) {
  ros1_request.rover_id = ros2_request.rover_id;
  ros1_request.recording_profile = ros2_request.recording_profile;
  ros1_request.name = ros2_request.name;
  ros1_request.mapping = ros2_request.mapping;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::srv::StartMission::Response, im_vps_api_v1_interfaces::StartMission::Response>(
    im_vps_api_v1_interfaces::StartMission::Response&& ros1_response,
    im_vps_api_v1_interfaces::srv::StartMission::Response& ros2_response) {
  ros2_response.mission_uuid = std::move(ros1_response.mission_uuid);
  convert_1_to_2(std::move(ros1_response.result), ros2_response.result);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::StartMapping::Request, im_vps_api_v1_interfaces::srv::StartMapping::Request>(
    const im_vps_api_v1_interfaces::srv::StartMapping::Request& ros2_request,
    im_vps_api_v1_interfaces::StartMapping::Request& ros1_request) {
  ros1_request.rover_id = ros2_request.rover_id;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::srv::StartMapping::Response, im_vps_api_v1_interfaces::StartMapping::Response>(
    im_vps_api_v1_interfaces::StartMapping::Response&& ros1_response,
    im_vps_api_v1_interfaces::srv::StartMapping::Response& ros2_response) {
  ros2_response.map_uuid = std::move(ros1_response.map_uuid);
  convert_1_to_2(std::move(ros1_response.result), ros2_response.result);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::StopMission::Request, im_vps_api_v1_interfaces::srv::StopMission::Request>(
    const im_vps_api_v1_interfaces::srv::StopMission::Request& ros2_request,
    im_vps_api_v1_interfaces::StopMission::Request& ros1_request) {
  ros1_request.rover_id = ros2_request.rover_id;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::srv::StopMission::Response, im_vps_api_v1_interfaces::StopMission::Response>(
    im_vps_api_v1_interfaces::StopMission::Response&& ros1_response,
    im_vps_api_v1_interfaces::srv::StopMission::Response& ros2_response) {
  convert_1_to_2(std::move(ros1_response.result), ros2_response.result);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::StopMapping::Request, im_vps_api_v1_interfaces::srv::StopMapping::Request>(
    const im_vps_api_v1_interfaces::srv::StopMapping::Request& ros2_request,
    im_vps_api_v1_interfaces::StopMapping::Request& ros1_request) {
  ros1_request.rover_id = ros2_request.rover_id;
  ros1_request.merge_with_loaded_map = ros2_request.merge_with_loaded_map;
  ros1_request.generate_localization_map = ros2_request.generate_localization_map;
  ros1_request.export_trajectory = ros2_request.export_trajectory;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::srv::StopMapping::Response, im_vps_api_v1_interfaces::StopMapping::Response>(
    im_vps_api_v1_interfaces::StopMapping::Response&& ros1_response,
    im_vps_api_v1_interfaces::srv::StopMapping::Response& ros2_response) {
  ros2_response.map_uuid = std::move(ros1_response.map_uuid);
  convert_1_to_2(std::move(ros1_response.result), ros2_response.result);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::SetMapMetadata::Request, im_vps_api_v1_interfaces::srv::SetMapMetadata::Request>(
    const im_vps_api_v1_interfaces::srv::SetMapMetadata::Request& ros2_request,
    im_vps_api_v1_interfaces::SetMapMetadata::Request& ros1_request) {
  ros1_request.rover_id = ros2_request.rover_id;
  ros1_request.map_uuid = ros2_request.map_uuid;
  convert_2_to_1(ros2_request.extra_data, ros1_request.extra_data);
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::srv::SetMapMetadata::Response, im_vps_api_v1_interfaces::SetMapMetadata::Response>(
    im_vps_api_v1_interfaces::SetMapMetadata::Response&& ros1_response,
    im_vps_api_v1_interfaces::srv::SetMapMetadata::Response& ros2_response) {
  convert_1_to_2(std::move(ros1_response.result), ros2_response.result);
}

template <>
inline void convert_2_to_1<im_vps_api_v1_interfaces::UnloadMap::Request, im_vps_api_v1_interfaces::srv::UnloadMap::Request>(
    const im_vps_api_v1_interfaces::srv::UnloadMap::Request& ros2_request,
    im_vps_api_v1_interfaces::UnloadMap::Request& ros1_request) {
  ros1_request.rover_id = ros2_request.rover_id;
}

template <>
inline void convert_1_to_2<im_vps_api_v1_interfaces::srv::UnloadMap::Response, im_vps_api_v1_interfaces::UnloadMap::Response>(
    im_vps_api_v1_interfaces::UnloadMap::Response&& ros1_response,
    im_vps_api_v1_interfaces::srv::UnloadMap::Response& ros2_response) {
  convert_1_to_2(std::move(ros1_response.result), ros2_response.result);
}

}  // namespace conversions
}  // namespace ros2in1_support

#endif  // ROS2_SUPPORT
#endif  // ROS2IN1_SUPPORT_CONVERSIONS_IM_VPS_API_V1_INTERFACES_H
