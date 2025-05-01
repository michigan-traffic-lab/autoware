// Copyright 2019 Autoware Foundation
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

#include "test_07.hpp"

namespace test_07{

  Test07::Test07(const rclcpp::NodeOptions & options)
  : Node("test_07", options)
  {
    pub_goal = this->create_publisher<PoseStamped>("/planning/mission_planning/goal", 10);
    pub_local = this->create_publisher<PoseWithCovarianceStamped>("/initialpose", 10);

    cli_set_operation_mode = this->create_client<ChangeOperationMode>("/system/operation_mode/change_operation_mode");
    cli_set_autoware_control = this->create_client<ChangeAutowareControl>("/system/operation_mode/change_autoware_control");

    sub_autoware_state = this->create_subscription<AutowareState>(
      "/autoware/state", 10, std::bind(&Test07::autoware_state_callback, this, std::placeholders::_1));

    timer_ = rclcpp::create_timer(
      this, get_clock(), 1000ms, std::bind(&Test07::on_timer, this));

    if (!redis_client.connect(true)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to Redis server.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Connected to Redis server.");
    }
  }

  void Test07::on_timer(){
    if (autoware_state == AutowareState::INITIALIZING){
      init_localization();
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for vehicle initialization...");
    }
    else if (autoware_state == AutowareState::WAITING_FOR_ROUTE){
      set_route_points();
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting route points...");
    }
    else if (autoware_state == AutowareState::WAITING_FOR_ENGAGE){
      std::string terasim_status_str = redis_client.get("terasim_status");

      if (terasim_status_str.empty()) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("rclcpp"), *get_clock(), 1000, "Terasim status not available, waiting...");
        return;
      }
      
      // Directly parse the string to integer
      int terasim_status = std::stoi(terasim_status_str);
      
      if (terasim_status == 0) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("rclcpp"), *get_clock(), 1000, "Terasim not ready, waiting...");
      } else {
        set_autoware_control(true);
        set_operation_mode(ChangeOperationMode::Request::AUTONOMOUS);
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("rclcpp"), *get_clock(), 1000, "Enabling autoware control...");
      }
    }
  }

  void Test07::init_localization(){
    localization_msg.pose.pose.position.x = 57.037353515625;
    localization_msg.pose.pose.position.y = 116.36869812011719;

    localization_msg.pose.pose.orientation.x = 0.0;
    localization_msg.pose.pose.orientation.y = 0.0;
    localization_msg.pose.pose.orientation.z = 0.7366898752107643;
    localization_msg.pose.pose.orientation.w = 0.6762307503818119;

    localization_msg.header.stamp = this->get_clock()->now();
    localization_msg.header.frame_id = "map";

    pub_local->publish(localization_msg);
  }

  void Test07::set_route_points(){
    goal_msg.pose.position.x = 76.82257080078125;
    goal_msg.pose.position.y = 275.37261962890625;

    goal_msg.pose.orientation.x = 0.0;
    goal_msg.pose.orientation.y = 0.0;
    goal_msg.pose.orientation.z = 0.42004345209862004;
    goal_msg.pose.orientation.w = 0.9075039935719701;

    goal_msg.header.stamp = this->get_clock()->now();
    goal_msg.header.frame_id = "map";

    pub_goal->publish(goal_msg);
  }

  void Test07::set_operation_mode(uint8_t mode){
    auto request = std::make_shared<ChangeOperationMode::Request>();
    request->mode = mode;

    while (!cli_set_operation_mode->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "routing service not available, waiting again...");
    }

    auto result = cli_set_operation_mode->async_send_request(request);
  }

  void Test07::set_autoware_control(bool autoware_control){
    auto request = std::make_shared<ChangeAutowareControl::Request>();
    request->autoware_control = autoware_control;

    while (!cli_set_autoware_control->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "routing service not available, waiting again...");
    }

    auto result = cli_set_autoware_control->async_send_request(request);
  }

  void Test07::autoware_state_callback(AutowareState::SharedPtr msg){
    autoware_state = msg->state;
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(test_07::Test07)
