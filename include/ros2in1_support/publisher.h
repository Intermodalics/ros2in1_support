/******************************************************************************
 *  Copyright (c) 2024, Intermodalics BVBA                                    *
 *  All rights reserved.                                                      *
 ******************************************************************************/

#ifndef ROS2IN1_SUPPORT_PUBLISHER_H
#define ROS2IN1_SUPPORT_PUBLISHER_H

#include <functional>
#include <memory>
#include <utility>

#include <ros/node_handle.h>
#include <ros/publisher.h>

#ifdef ROS2_SUPPORT
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

#include "node.h"

namespace ros2in1_support {

namespace conversions {
template <typename Ros1MessageT> struct Ros2MessageType;
}  // namespace conversions

template <typename Ros1MessageT,
          typename Ros2MessageT = typename conversions::Ros2MessageType<Ros1MessageT>::type>
class Publisher {
 public:
  void advertise(
      ros::NodeHandle& ros1_node,
      const std::string& topic,
      uint32_t queue_size, bool latch = false) {
    advertise(ros1_node, getRos2Node(ros1_node), topic, queue_size, latch);
  }

  void advertise(
      ros::NodeHandle& ros1_node,
      const rclcpp::Node::SharedPtr& ros2_node,
      const std::string& topic,
      uint32_t queue_size, bool latch = false) {
    ros1_publisher_ = ros1_node.advertise<Ros1MessageT>(topic, queue_size, latch);

    rclcpp::QoS qos(queue_size);
    if (latch) qos = qos.transient_local();
    ros2_publisher_ = ros2_node->create_publisher<Ros2MessageT>(topic, qos);
  }

  void publish(const Ros1MessageT& ros1_msg) const {
    if (ros2_publisher_ && (ros2_publisher_->get_subscription_count())) {
      ros2_publisher_->publish(conversions::convert_1_to_2<Ros2MessageT>(ros1_msg));
    }
    ros1_publisher_.publish(ros1_msg);
  }

  void publish(boost::shared_ptr<Ros1MessageT> ros1_msg) const {
    if (ros2_publisher_ && (ros2_publisher_->get_subscription_count())) {
      ros2_publisher_->publish(conversions::convert_1_to_2<Ros2MessageT>(*ros1_msg));
    }
    ros1_publisher_.publish(std::move(ros1_msg));
  }

  size_t getNumSubscribers() const {
    std::size_t num_subscribers =
        static_cast<size_t>(ros1_publisher_.getNumSubscribers());
    if (ros2_publisher_) {
      num_subscribers += ros2_publisher_->get_subscription_count();
    }
    return num_subscribers;
  }

  explicit operator bool() const {
    return ros1_publisher_ && ros2_publisher_;
  }

 private:
  ros::Publisher ros1_publisher_;
  typename rclcpp::Publisher<Ros2MessageT>::SharedPtr ros2_publisher_;
};

#else  // ROS2_SUPPORT

namespace ros2in1_support {

template <typename Ros1MessageT>
class Publisher {
 public:
  void advertise(
      ros::NodeHandle& ros1_node,
      const std::string &topic,
      uint32_t queue_size, bool latch = false) {
    advertise(ros1_node, nullptr, topic, queue_size, latch);
  }

  void advertise(
      ros::NodeHandle& ros1_node,
      const std::shared_ptr<void>&,
      const std::string &topic,
      uint32_t queue_size, bool latch = false) {
    ros1_publisher_ = ros1_node.advertise<Ros1MessageT>(topic, queue_size, latch);
  }

  void publish(const Ros1MessageT& ros1_msg) const {
    ros1_publisher_.publish(ros1_msg);
  }

  void publish(boost::shared_ptr<Ros1MessageT> ros1_msg) const {
    ros1_publisher_.publish(std::move(ros1_msg));
  }

  size_t getNumSubscribers() const {
    return static_cast<size_t>(ros1_publisher_.getNumSubscribers());
  }

  explicit operator bool() const {
    return ros1_publisher_;
  }

 private:
  ros::Publisher ros1_publisher_;
  std::shared_ptr<void> ros2_publisher_;  // for identical memory layout independent of ROS2_SUPPORT
};

#endif  // ROS2_SUPPORT

}  // namespace ros2in1_support

# endif  // ROS2IN1_SUPPORT_PUBLISHER_H
