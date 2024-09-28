// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DYNAMIXEL_HW_INTERFACE_NODE_HPP
#define DYNAMIXEL_HW_INTERFACE_NODE_HPP

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
// #include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
// #include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

// Control table address for XC430-W240(T240BB)
// Refer to : https://emanual.robotis.com/docs/en/dxl/x/xc430-w240/#control-table
struct Dynamixel_Motor_XC430
{
  uint8_t id;
  uint32_t profile_velocity; // In units of 0.229 RPM
  int32_t goal_velocity;     // In units of 0.229 RPM
  int32_t min_position_limit;
  int32_t max_position_limit;

  static const uint32_t BAUDRATE = 1000000;
  static constexpr float PROTOCOL_VERSION = 2.0;

  static const uint16_t ADDR_OPERATING_MODE = 11;
  static const uint16_t ADDR_TORQUE_ENABLE = 64;
  static const uint16_t ADDR_GOAL_POSITION = 116;
  static const uint16_t ADDR_PRESENT_POSITION = 132;
  static const uint16_t ADDR_PROFILE_VELOCITY = 112;
  static const uint16_t ADDR_GOAL_VELOCITY = 104;
  static const uint8_t EXTENDED_POSITION_CONTROL_MODE = 4;
};

// class DynamixelHwInterfaceNode  : public rclcpp::Node
// {
// public:
//   using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
//   using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;

//   DynamixelHwInterfaceNode ();
//   virtual ~DynamixelHwInterfaceNode ();

// private:
//   rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
//   rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
//   int present_position;
// };

#endif // DYNAMIXEL_HW_INTERFACE_NODE_HPP
