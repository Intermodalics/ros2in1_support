/******************************************************************************
 *  Copyright (c) 2024, Intermodalics BVBA                                    *
 *  All rights reserved.                                                      *
 ******************************************************************************/

#ifndef ROS2IN1_SUPPORT_SERVICE_SERVER_H
#define ROS2IN1_SUPPORT_SERVICE_SERVER_H

#include <functional>
#include <memory>
#include <utility>

#include <ros/node_handle.h>
#include <ros/service_server.h>

#ifdef ROS2_SUPPORT
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>

#include "node.h"

namespace ros2in1_support {

template <typename Ros1ServiceT,
          typename Ros2ServiceT = typename conversions::Ros2ServiceType<Ros1ServiceT>::type>
class ServiceServer {
 public:
  typedef bool CallbackSignature(typename Ros1ServiceT::Request&, typename Ros1ServiceT::Response&);
  typedef std::function<CallbackSignature> Callback;

  void advertise(
      ros::NodeHandle& ros1_node,
      const std::string &service, Callback&& callback) {
    advertise(ros1_node, getRos2Node(ros1_node), service, std::move(callback));
  }

  void advertise(
      ros::NodeHandle& ros1_node,
      const rclcpp::Node::SharedPtr& ros2_node,
      const std::string &service, Callback&& callback) {
    callback_ = std::move(callback);
    ros1_service_ = ros1_node.advertiseService<ServiceServer, typename Ros1ServiceT::Request, typename Ros1ServiceT::Response>(
        service, &ServiceServer::callbackRos1, this);
    ros2_service_ = ros2_node->create_service<Ros2ServiceT>(
        service, std::bind(
            &ServiceServer::callbackRos2, this,
            std::placeholders::_1, std::placeholders::_2));
  }

  template <class T>
  void advertise(
      ros::NodeHandle& ros1_node, const std::string &service,
      bool(T::*srv_func)(typename Ros1ServiceT::Request&, typename Ros1ServiceT::Response&), T *obj) {
    advertise(
        ros1_node, service,
        std::bind(srv_func, obj, std::placeholders::_1, std::placeholders::_2));
  }

  template <class T>
  void advertise(
      ros::NodeHandle& ros1_node,
      const rclcpp::Node::SharedPtr& ros2_node,
      const std::string &service,
      bool(T::*srv_func)(typename Ros1ServiceT::Request&, typename Ros1ServiceT::Response&), T *obj) {
    advertise(
        ros1_node, ros2_node, service,
        std::bind(srv_func, obj, std::placeholders::_1, std::placeholders::_2));
  }

  bool callbackRos1(
      typename Ros1ServiceT::Request& ros1_request,
      typename Ros1ServiceT::Response& ros1_response) {
    return callback_(ros1_request, ros1_response);
  }

  void callbackRos2(
      const std::shared_ptr<typename Ros2ServiceT::Request> ros2_request,
      std::shared_ptr<typename Ros2ServiceT::Response> ros2_response) {
    Ros1ServiceT ros1_service;
    conversions::convert_2_to_1<typename Ros1ServiceT::Request, typename Ros2ServiceT::Request>(
        *ros2_request, ros1_service.request);
    if (!callback_(ros1_service.request, ros1_service.response)) {
      throw std::runtime_error("service callback returned false");
    }
    conversions::convert_1_to_2<typename Ros2ServiceT::Response, typename Ros1ServiceT::Response>(
        std::move(ros1_service.response), *ros2_response);
  }

  explicit operator bool() const {
    return ros1_service_ && ros2_service_;
  }

 private:
  ros::ServiceServer ros1_service_;
  typename rclcpp::Service<Ros2ServiceT>::SharedPtr ros2_service_;
  Callback callback_;
};

#else  // ROS2_SUPPORT

namespace ros2in1_support {

template <typename Ros1ServiceT>
class ServiceServer {
 public:
  typedef bool CallbackSignature(typename Ros1ServiceT::Request&, typename Ros1ServiceT::Response&);
  typedef std::function<CallbackSignature> Callback;

  void advertise(
      ros::NodeHandle& ros1_node,
      const std::string &service, Callback&& callback) {
    advertise(ros1_node, nullptr, service, std::move(callback));
  }

  void advertise(
      ros::NodeHandle& ros1_node,
      const std::shared_ptr<void>&,
      const std::string &service, Callback&& callback) {
    callback_ = std::move(callback);
    ros1_service_ = ros1_node.advertiseService<ServiceServer, typename Ros1ServiceT::Request, typename Ros1ServiceT::Response>(
        service, &ServiceServer::callbackRos1, this);
  }

  template <class T>
  void advertise(
      ros::NodeHandle& ros1_node, const std::string &service,
      bool(T::*srv_func)(typename Ros1ServiceT::Request&, typename Ros1ServiceT::Response&), T *obj) {
    advertise(
        ros1_node, nullptr, service,
        std::bind(srv_func, obj, std::placeholders::_1, std::placeholders::_2));
  }

  template <class T>
  void advertise(
      ros::NodeHandle& ros1_node,
      const std::shared_ptr<void>&,
      const std::string &service,
      bool(T::*srv_func)(typename Ros1ServiceT::Request&, typename Ros1ServiceT::Response&), T *obj) {
    advertise(
        ros1_node, nullptr, service,
        std::bind(srv_func, obj, std::placeholders::_1, std::placeholders::_2));
  }

  bool callbackRos1(
      typename Ros1ServiceT::Request& ros1_request,
      typename Ros1ServiceT::Response& ros1_response) {
    if (!callback_) return false;
    return callback_(ros1_request, ros1_response);
  }

  explicit operator bool() const {
    return ros1_service_;
  }

 private:
  ros::ServiceServer ros1_service_;
  std::shared_ptr<void> ros2_service_;  // for identical memory layout independent of ROS2_SUPPORT
  Callback callback_;
};

#endif  // ROS2_SUPPORT

}  // namespace ros2in1_support

#endif  // ROS2IN1_SUPPORT_SERVICE_SERVER_H
