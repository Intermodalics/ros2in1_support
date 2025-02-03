/******************************************************************************
 *  Copyright (c) 2024, Intermodalics BVBA                                    *
 *  All rights reserved.                                                      *
 ******************************************************************************/

#include "ros2in1_support/tf.h"

#include <utility>

#include <glog/logging.h>
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

void publishTf(
    const aslam::Transformation& T, const std::string& frame_id,
    const std::string& child_frame_id, const ros::Time& ros_time,
    geometry_msgs::TransformStamped* transform_msg,
    const bool project_to_2d) {
  CHECK(!frame_id.empty());
  CHECK(!child_frame_id.empty());

  geometry_msgs::TransformStamped transform_msg_tmp;
  if (transform_msg == nullptr) transform_msg = &transform_msg_tmp;

  transform_msg->header.frame_id = frame_id;
  transform_msg->header.stamp = ros_time;
  transform_msg->child_frame_id = child_frame_id;
  transform_msg->transform.translation.x = T.getPosition().x();
  transform_msg->transform.translation.y = T.getPosition().y();
  transform_msg->transform.translation.z = T.getPosition().z();
  transform_msg->transform.rotation.x = T.getRotation().x();
  transform_msg->transform.rotation.y = T.getRotation().y();
  transform_msg->transform.rotation.z = T.getRotation().z();
  transform_msg->transform.rotation.w = T.getRotation().w();

  if (project_to_2d) {
    transform_msg->transform.translation.z = 0.0;
    double yaw = tf2::getYaw(transform_msg->transform.rotation);
    tf2::Quaternion orientation_yaw_only(tf2::Vector3(0.0, 0.0, 1.0), yaw);
    transform_msg->transform.rotation = tf2::toMsg(orientation_yaw_only);
  }

  TransformBroadcaster::getInstance().sendTransform(*transform_msg);
}

void publishTf(
    const aslam::Transformation& T, const std::string& frame_id,
    const std::string& child_frame_id, geometry_msgs::TransformStamped* transform_msg) {
  publishTf(T, frame_id, child_frame_id, ros::Time::now(), transform_msg);
}

}  // namespace ros2in1_support
