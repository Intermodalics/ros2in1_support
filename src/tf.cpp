/******************************************************************************
 *  Copyright (c) 2024, Intermodalics BVBA                                    *
 *  All rights reserved.                                                      *
 ******************************************************************************/

#include "ros2in1_support/tf.h"

#include <utility>

#include <tf2/utils.h>

#ifdef ROS2_SUPPORT
#include "ros2in1_support/conversions/geometry_msgs.h"
#include "ros2in1_support/node.h"
#endif  // ROS2_SUPPORT

namespace ros2in1_support {

const TransformBroadcaster& TransformBroadcaster::getInstance() {
  static TransformBroadcaster instance;
  return instance;
}

TransformBroadcaster::TransformBroadcaster()
    : TransformBroadcaster(ros::NodeHandle()) {}

TransformBroadcaster::TransformBroadcaster(
    const ros::NodeHandle& ros1_node,
#ifdef ROS2_SUPPORT
    rclcpp::Node::SharedPtr ros2_node)
    : ros1_node_(ros1_node) {
  if (!ros2_node) ros2_node = getRos2Node(ros1_node_);
  ros2_publisher_ = ros2_node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
#else
    std::shared_ptr<void>)
    : ros1_node_(ros1_node) {
#endif
  ros1_publisher_ = ros1_node_.advertise<tf2_msgs::TFMessage>("/tf", 100);
}

void TransformBroadcaster::sendTransform(std::vector<geometry_msgs::TransformStamped> transforms) const {
#ifdef ROS2_SUPPORT
  if (ros2_publisher_) {
    tf2_msgs::msg::TFMessage tf_message;
    conversions::convert_1_to_2<geometry_msgs::msg::TransformStamped>(
        transforms, tf_message.transforms);
    ros2_publisher_->publish(tf_message);
  }
#endif
  if (ros1_publisher_) {
    tf2_msgs::TFMessage tf_message;
    tf_message.transforms = std::move(transforms);
    ros1_publisher_.publish(tf_message);
  }
}

void TransformBroadcaster::sendTransform(const geometry_msgs::TransformStamped& transform) const {
  sendTransform(std::vector<geometry_msgs::TransformStamped>{transform});
}

void TransformBroadcaster::sendTransform(geometry_msgs::TransformStamped&& transform) const {
  std::vector<geometry_msgs::TransformStamped> transforms;
  transforms.emplace_back(std::move(transform));
  sendTransform(std::move(transforms));
}

void publishTf(std::vector<geometry_msgs::TransformStamped> transforms) {
  TransformBroadcaster::getInstance().sendTransform(std::move(transforms));
}

void publishTf(const geometry_msgs::TransformStamped& transform) {
  TransformBroadcaster::getInstance().sendTransform(transform);
}

void publishTf(geometry_msgs::TransformStamped&& transform) {
  TransformBroadcaster::getInstance().sendTransform(std::move(transform));
}

}  // namespace ros2in1_support
