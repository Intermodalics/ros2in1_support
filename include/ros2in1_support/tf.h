/******************************************************************************
 *  Copyright (c) 2024, Intermodalics BVBA                                    *
 *  All rights reserved.                                                      *
 ******************************************************************************/

#ifndef ROS2IN1_SUPPORT_TF_H
#define ROS2IN1_SUPPORT_TF_H

#include <memory>
#include <vector>

#include <aslam/common/pose-types.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <tf2_msgs/TFMessage.h>

#ifdef ROS2_SUPPORT
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "conversions/geometry_msgs.h"
#endif  // ROS2_SUPPORT

namespace ros2in1_support {

class TransformBroadcaster {
 public:
  static const TransformBroadcaster& getInstance();

  TransformBroadcaster();
  TransformBroadcaster(
      const ros::NodeHandle& ros1_node,
#ifdef ROS2_SUPPORT
      rclcpp::Node::SharedPtr ros2_node = nullptr);
#else
      std::shared_ptr<void> = nullptr);
#endif

  void sendTransform(std::vector<geometry_msgs::TransformStamped> transforms) const;
  void sendTransform(const geometry_msgs::TransformStamped& transform) const;
  void sendTransform(geometry_msgs::TransformStamped&& transform) const;

 private:
  ros::NodeHandle ros1_node_;
  ros::Publisher ros1_publisher_;
#ifdef ROS2_SUPPORT
  typename rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr ros2_publisher_;
#else
  std::shared_ptr<void> ros2_publisher_;
#endif
};

void publishTf(std::vector<geometry_msgs::TransformStamped> transforms);
void publishTf(const geometry_msgs::TransformStamped& transform);
void publishTf(geometry_msgs::TransformStamped&& transform);

void publishTf(
    const aslam::Transformation& T, const std::string& frame_id,
    const std::string& child_frame_id, const ros::Time& ros_time,
    geometry_msgs::TransformStamped* transform_msg,
    const bool project_to_2d = false);

void publishTf(
    const aslam::Transformation& T, const std::string& frame_id,
    const std::string& child_frame_id, geometry_msgs::TransformStamped* transform_msg);

}  // namespace ros2in1_support

#endif  // ROS2IN1_SUPPORT_TF_H
