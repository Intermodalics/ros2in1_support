/******************************************************************************
 *  Copyright (c) 2025, Intermodalics BVBA                                    *
 *  All rights reserved.                                                      *
 ******************************************************************************/

#ifndef ROS2IN1_SUPPORT_SUBSCRIBER_H
#define ROS2IN1_SUPPORT_SUBSCRIBER_H

#include <functional>
#include <memory>
#include <utility>

#include <ros/node_handle.h>
#include <ros/subscriber.h>

#ifdef ROS2_SUPPORT
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>

#include "node.h"

namespace ros2in1_support {

namespace conversions {
template <typename Ros1MessageT> struct Ros2MessageType;
}  // namespace conversions

namespace transport_hint_queries {
inline bool isReliable(ros::TransportHints& transport_hints) {
  const auto& transports = transport_hints.getTransports();
  return (std::find(transports.begin(), transports.end(), "TCP")
          != transports.end());
}

inline bool isUnreliable(ros::TransportHints& transport_hints) {
  const auto& transports = transport_hints.getTransports();
  return (std::find(transports.begin(), transports.end(), "UDP")
          != transports.end());
}
}

template <typename Ros1MessageT,
          typename Ros2MessageT = typename conversions::Ros2MessageType<Ros1MessageT>::type>
class Subscriber {
 public:

private:
rclcpp::QoS make_qos_from_transport_hints(uint32_t queue_size, ros::TransportHints transport_hints) const {
  auto reliability = rclcpp::ReliabilityPolicy::SystemDefault;
  if (transport_hint_queries::isReliable(transport_hints)) {
    reliability = rclcpp::ReliabilityPolicy::Reliable;
  } else if (transport_hint_queries::isUnreliable(transport_hints)) {
    reliability = rclcpp::ReliabilityPolicy::BestEffort;
  }
  return rclcpp::QoS(queue_size).reliability(reliability);
}

public:
/** First implementation: set up for ROS 2 and ROS 1 */
template<class M, class T>
void subscribe(ros::NodeHandle& ros1_node, const std::string& topic, uint32_t queue_size, void(T::*fp)(M), T* obj, 
  const ros::TransportHints& transport_hints = ros::TransportHints())
{
  // Set up ROS 2 subscription with a lambda that converts and calls the ROS 1-style callback
  rclcpp::Node::SharedPtr node = ros2in1_support::getRos2Node();
  ros2_subscription_ = node->create_subscription<Ros2MessageT>(
    topic, make_qos_from_transport_hints(queue_size, transport_hints),
    [fp, obj](const typename Ros2MessageT::SharedPtr ros2_msg) {
      // Convert ROS 2 message to ROS 1 message
      Ros1MessageT ros1_msg;
      conversions::convert_2_to_1(*ros2_msg, ros1_msg);
      // Call the ROS 1-style callback
      (obj->*fp)(ros1_msg);
    });

  // Full forward to ROS 1.
  ros1_subscriber_ = ros1_node.subscribe<Ros1MessageT>(
    topic, queue_size, fp, obj, transport_hints);
}

// const fp version of the above
template<class M, class T>
void subscribe(ros::NodeHandle& ros1_node, const std::string& topic, uint32_t queue_size, void(T::*fp)(M) const, T* obj, 
    const ros::TransportHints& transport_hints = ros::TransportHints())
{
  // Set up ROS 2 subscription with a lambda that converts and calls the ROS 1-style const callback
  rclcpp::Node::SharedPtr node = ros2in1_support::getRos2Node();
  ros2_subscription_ = node->create_subscription<Ros2MessageT>(
    topic, make_qos_from_transport_hints(queue_size, transport_hints),
    [fp, obj](const typename Ros2MessageT::SharedPtr ros2_msg) {
      Ros1MessageT ros1_msg;
      conversions::convert_2_to_1(*ros2_msg, ros1_msg);
      // Call the ROS 1-style const callback
      (obj->*fp)(ros1_msg);
    });

  // Full forward to ROS 1.
  ros1_subscriber_ = ros1_node.subscribe<Ros1MessageT>(
    topic, queue_size, fp, obj, transport_hints);
}

template<class M, class T>
void subscribe(ros::NodeHandle& ros1_node, const std::string& topic, uint32_t queue_size, 
    void(T::*fp)(const boost::shared_ptr<M const>&), T* obj, 
    const ros::TransportHints& transport_hints = ros::TransportHints())
{
  // Set up ROS 2 subscription with a lambda that converts and calls the ROS 1-style const callback
  rclcpp::Node::SharedPtr node = ros2in1_support::getRos2Node();
  ros2_subscription_ = node->create_subscription<Ros2MessageT>(
    topic, make_qos_from_transport_hints(queue_size, transport_hints),
    [fp, obj](const typename Ros2MessageT::SharedPtr ros2_msg) {
      boost::shared_ptr<Ros1MessageT> ros1_msg = boost::make_shared<Ros1MessageT>();
      conversions::convert_2_to_1(*ros2_msg, *ros1_msg);
      // Call the ROS 1-style const callback
      (obj->*fp)(ros1_msg);
    });

  // Full forward to ROS 1.
  ros1_subscriber_ = ros1_node.subscribe<Ros1MessageT>(
    topic, queue_size, fp, obj, transport_hints);
}
template<class M, class T>
void subscribe(ros::NodeHandle& ros1_node, const std::string& topic, uint32_t queue_size, 
    void(T::*fp)(const boost::shared_ptr<M const>&) const, T* obj, 
    const ros::TransportHints& transport_hints = ros::TransportHints())
{
  // Set up ROS 2 subscription with a lambda that converts and calls the ROS 1-style const callback
  rclcpp::Node::SharedPtr node = ros2in1_support::getRos2Node();
  ros2_subscription_ = node->create_subscription<Ros2MessageT>(
    topic, make_qos_from_transport_hints(queue_size, transport_hints),
    [fp, obj](const typename Ros2MessageT::SharedPtr ros2_msg) {
      boost::shared_ptr<Ros1MessageT> ros1_msg = boost::make_shared<Ros1MessageT>();
      conversions::convert_2_to_1(*ros2_msg, *ros1_msg);
      // Call the ROS 1-style const callback
      (obj->*fp)(ros1_msg);
    });

  // Full forward to ROS 1.
  ros1_subscriber_ = ros1_node.subscribe<Ros1MessageT>(
    topic, queue_size, fp, obj, transport_hints);
}
#if 0
  template<class M, class T>
  void subscribe(const std::string& topic, uint32_t queue_size, void(T::*fp)(M), 
                       const boost::shared_ptr<T>& obj, const ros::TransportHints& transport_hints = ros::TransportHints())
  {

  }

  template<class M, class T>
  void subscribe(const std::string& topic, uint32_t queue_size, void(T::*fp)(M) const, 
                       const boost::shared_ptr<T>& obj, const ros::TransportHints& transport_hints = ros::TransportHints())
  {

  }

  template<class M, class T>
  void subscribe(const std::string& topic, uint32_t queue_size, 
                       void(T::*fp)(const boost::shared_ptr<M const>&), 
                       const boost::shared_ptr<T>& obj, const ros::TransportHints& transport_hints = ros::TransportHints())
  {

  }
  template<class M, class T>
  void subscribe(const std::string& topic, uint32_t queue_size, 
                       void(T::*fp)(const boost::shared_ptr<M const>&) const, 
                       const boost::shared_ptr<T>& obj, const ros::TransportHints& transport_hints = ros::TransportHints())
  {

  }

  template<class M>
  void subscribe(const std::string& topic, uint32_t queue_size, void(*fp)(M), const ros::TransportHints& transport_hints = ros::TransportHints())
  {

  }

  template<class M>
  void subscribe(const std::string& topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr<M const>&), const ros::TransportHints& transport_hints = ros::TransportHints())
  {

  }

  template<class M>
  void subscribe(const std::string& topic, uint32_t queue_size, const boost::function<void (const boost::shared_ptr<M const>&)>& callback,
                             const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(), const ros::TransportHints& transport_hints = ros::TransportHints())
  {

  }

  template<class M, class C>
  void subscribe(const std::string& topic, uint32_t queue_size, const boost::function<void (C)>& callback,
                             const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(), const ros::TransportHints& transport_hints = ros::TransportHints())
  {

  }
#endif

  explicit operator bool() const {
    return ros1_subscriber_ && ros2_subscription_;
  }

 private:
  ros::Subscriber ros1_subscriber_;
  typename rclcpp::Subscription<Ros2MessageT>::SharedPtr ros2_subscription_;
};

#else  // ROS2_SUPPORT

namespace ros2in1_support {

template <typename Ros1MessageT>
class Subscriber {
 public:
 /// TODO Add the wrappers to ROS 1 API

  explicit operator bool() const {
    return ros1_publisher_;
  }

 private:
  ros::Subscriber ros1_subscriber_;
  std::shared_ptr<void> ros2_subscription_;  // for identical memory layout independent of ROS2_SUPPORT
};

#endif  // ROS2_SUPPORT

}  // namespace ros2in1_support

# endif  // ROS2IN1_SUPPORT_PUBLISHER_H
