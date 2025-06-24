/******************************************************************************
 *  Copyright (c) 2024, Intermodalics BVBA                                    *
 *  All rights reserved.                                                      *
 ******************************************************************************/

#ifndef ROS2IN1_SUPPORT_CONVERSIONS_COMMON_H
#define ROS2IN1_SUPPORT_CONVERSIONS_COMMON_H
#ifdef ROS2_SUPPORT

#include <array>
#include <utility>
#include <vector>

#include <boost/array.hpp>

namespace ros2in1_support {
namespace conversions {

template <typename Ros2MessageT, typename Ros1MessageT>
__attribute__((unused)) inline void convert_1_to_2(const Ros1MessageT& ros1_msg, Ros2MessageT& ros2_msg);

template <typename Ros2MessageT, typename Ros1MessageT>
inline void convert_1_to_2(Ros1MessageT&& ros1_msg, Ros2MessageT& ros2_msg) {
  convert_1_to_2<Ros2MessageT, Ros1MessageT>(static_cast<const Ros1MessageT&>(ros1_msg), ros2_msg);
}

template <typename Ros1MessageT, typename Ros2MessageT>
__attribute__((unused)) inline void convert_2_to_1(const Ros2MessageT& ros2_msg, Ros1MessageT& ros1_msg);

template <typename Ros1MessageT, typename Ros2MessageT>
inline void convert_2_to_1(Ros2MessageT&& ros2_msg, Ros1MessageT& ros1_msg) {
  convert_2_to_1<Ros1MessageT, Ros2MessageT>(static_cast<const Ros2MessageT&>(ros2_msg), ros1_msg);
}

template <typename Ros2MessageT, typename Ros1MessageT>
inline Ros2MessageT convert_1_to_2(const Ros1MessageT& ros1_msg) {
  Ros2MessageT ros2_msg;
  ::ros2in1_support::conversions::convert_1_to_2<Ros2MessageT, Ros1MessageT>(ros1_msg, ros2_msg);
  return ros2_msg;
}

template <typename Ros1MessageT, typename Ros2MessageT>
inline Ros1MessageT convert_2_to_1(const Ros2MessageT& ros2_msg) {
  Ros1MessageT ros1_msg;
  ::ros2in1_support::conversions::convert_2_to_1<Ros1MessageT, Ros2MessageT>(ros2_msg, ros1_msg);
  return ros1_msg;
}

template <typename Ros2MessageT, typename Ros1MessageT>
inline void convert_1_to_2(
    const std::vector<Ros1MessageT>& ros1_msg,
    std::vector<Ros2MessageT>& ros2_msg) {
  ros2_msg.resize(ros1_msg.size());
  auto ros1_it = ros1_msg.begin();
  auto ros2_it = ros2_msg.begin();
  for( ; ros1_it != ros1_msg.end(); ++ros1_it, ++ros2_it) {
    ::ros2in1_support::conversions::convert_1_to_2<Ros2MessageT, Ros1MessageT>(*ros1_it, *ros2_it);
  }
}

template <typename Ros2MessageT, typename Ros1MessageT>
inline void convert_1_to_2(
    std::vector<Ros1MessageT>&& ros1_msg,
    std::vector<Ros2MessageT>& ros2_msg) {
  ros2_msg.resize(ros1_msg.size());
  auto ros1_it = ros1_msg.begin();
  auto ros2_it = ros2_msg.begin();
  for( ; ros1_it != ros1_msg.end(); ++ros1_it, ++ros2_it) {
    ::ros2in1_support::conversions::convert_1_to_2<Ros2MessageT, Ros1MessageT>(std::move(*ros1_it), *ros2_it);
  }
}

template <typename T>
inline void convert_1_to_2(
    const std::vector<T>& ros1_msg,
    std::vector<T>& ros2_msg) {
  ros2_msg = ros1_msg;
}

template <typename T>
inline void convert_1_to_2(
    std::vector<T>&& ros1_msg,
    std::vector<T>& ros2_msg) {
  ros2_msg = std::move(ros1_msg);
}

template <typename Ros1MessageT, typename Ros2MessageT>
inline void convert_2_to_1(
  const std::vector<Ros2MessageT>& ros2_msg,
  std::vector<Ros1MessageT>& ros1_msg) {
  ros1_msg.resize(ros2_msg.size());
  auto ros1_it = ros1_msg.begin();
  auto ros2_it = ros2_msg.begin();
  for( ; ros2_it != ros2_msg.end(); ++ros1_it, ++ros2_it) {
    ::ros2in1_support::conversions::convert_2_to_1<Ros1MessageT, Ros2MessageT>(*ros2_it, *ros1_it);
  }
}

template <typename Ros1MessageT, typename Ros2MessageT>
inline void convert_2_to_1(
  std::vector<Ros2MessageT>&& ros2_msg,
  std::vector<Ros1MessageT>& ros1_msg) {
  ros1_msg.resize(ros2_msg.size());
  auto ros1_it = ros1_msg.begin();
  auto ros2_it = ros2_msg.begin();
  for( ; ros2_it != ros2_msg.end(); ++ros1_it, ++ros2_it) {
    ::ros2in1_support::conversions::convert_2_to_1<Ros1MessageT, Ros2MessageT>(std::move(*ros2_it), *ros1_it);
  }
}

template <typename T>
inline void convert_2_to_1(
  const std::vector<T>& ros2_msg,
  std::vector<T>& ros1_msg) {
  ros1_msg = ros2_msg;
}

template <typename T>
inline void convert_2_to_1(
  std::vector<T>&& ros2_msg,
  std::vector<T>& ros1_msg) {
  ros1_msg = std::move(ros2_msg);
}

template <typename Ros2MessageT, typename Ros1MessageT, std::size_t N>
inline void convert_1_to_2(
    const boost::array<Ros1MessageT, N>& ros1_msg,
    std::array<Ros2MessageT, N>& ros2_msg) {
  auto ros1_it = ros1_msg.begin();
  auto ros2_it = ros2_msg.begin();
  for( ; ros1_it != ros1_msg.end(); ++ros1_it, ++ros2_it) {
    ::ros2in1_support::conversions::convert_1_to_2<Ros2MessageT, Ros1MessageT>(*ros1_it, *ros2_it);
  }
}

template <typename Ros2MessageT, typename Ros1MessageT, std::size_t N>
inline void convert_1_to_2(
    boost::array<Ros1MessageT, N>&& ros1_msg,
    std::array<Ros2MessageT, N>& ros2_msg) {
  auto ros1_it = ros1_msg.begin();
  auto ros2_it = ros2_msg.begin();
  for( ; ros1_it != ros1_msg.end(); ++ros1_it, ++ros2_it) {
    ::ros2in1_support::conversions::convert_1_to_2<Ros2MessageT, Ros1MessageT>(std::move(*ros1_it), *ros2_it);
  }
}

template <typename T, std::size_t N>
inline void convert_1_to_2(
    const boost::array<T, N>& ros1_msg,
    std::array<T, N>& ros2_msg) {
  std::copy(ros1_msg.begin(), ros1_msg.end(), ros2_msg.begin());
}

template <typename T, std::size_t N>
inline void convert_1_to_2(
    boost::array<T, N>&& ros1_msg,
    std::array<T, N>& ros2_msg) {
  std::move(ros1_msg.begin(), ros1_msg.end(), ros2_msg.begin());
}

template <typename Ros1MessageT, typename Ros2MessageT, std::size_t N>
inline void convert_2_to_1(
  const std::array<Ros2MessageT, N>& ros2_msg,
  boost::array<Ros1MessageT, N>& ros1_msg) {
  auto ros1_it = ros1_msg.begin();
  auto ros2_it = ros2_msg.begin();
  for( ; ros2_it != ros2_msg.end(); ++ros1_it, ++ros2_it) {
    ::ros2in1_support::conversions::convert_2_to_1<Ros1MessageT, Ros2MessageT>(*ros2_it, *ros1_it);
  }
}

template <typename Ros1MessageT, typename Ros2MessageT, std::size_t N>
inline void convert_2_to_1(
  std::array<Ros2MessageT, N>&& ros2_msg,
  boost::array<Ros1MessageT, N>& ros1_msg) {
  auto ros1_it = ros1_msg.begin();
  auto ros2_it = ros2_msg.begin();
  for( ; ros2_it != ros2_msg.end(); ++ros1_it, ++ros2_it) {
    ::ros2in1_support::conversions::convert_2_to_1<Ros1MessageT, Ros2MessageT>(std::move(*ros2_it), *ros1_it);
  }
}

template <typename T, std::size_t N>
inline void convert_2_to_1(
  const std::array<T, N>& ros2_msg,
  boost::array<T, N>& ros1_msg) {
  std::copy(ros2_msg.begin(), ros2_msg.end(), ros1_msg.begin());
}

template <typename T, std::size_t N>
inline void convert_2_to_1(
  std::array<T, N>&& ros2_msg,
  boost::array<T, N>& ros1_msg) {
  std::move(ros2_msg.begin(), ros2_msg.end(), ros1_msg.begin());
}

}  // namespace conversions
}  // namespace ros2in1_support

#endif  // ROS2_SUPPORT
#endif  // ROS2IN1_SUPPORT_CONVERSIONS_COMMON_H
