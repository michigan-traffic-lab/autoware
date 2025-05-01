#ifndef MCITY_DEMO_AUTOWARE_INTERFACE_DEMO_REALCAR_HPP_
#define MCITY_DEMO_AUTOWARE_INTERFACE_DEMO_REALCAR_HPP_

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <mcity_msgs/msg/vehicle_state.hpp>

#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>

#include <tier4_system_msgs/srv/change_operation_mode.hpp>
#include <tier4_system_msgs/srv/change_autoware_control.hpp>

#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_system_msgs/msg/autoware_state.hpp>


namespace autoware_interface_demo_realcar
{

using namespace std;

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using autoware_auto_system_msgs::msg::AutowareState;
using mcity_msgs::msg::VehicleState;
using autoware_adapi_v1_msgs::srv::SetRoutePoints;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using tier4_system_msgs::srv::ChangeOperationMode;
using tier4_system_msgs::srv::ChangeAutowareControl;
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using autoware_auto_vehicle_msgs::msg::SteeringReport;


class AutowareInterfaceDemoRealcar : public rclcpp::Node
{
public:
  explicit AutowareInterfaceDemoRealcar(const rclcpp::NodeOptions & options);
  ~AutowareInterfaceDemoRealcar() = default;

private:
  int autoware_state = 1;

  const double STEER_TO_TIRE_RATIO = 16.0;

  VehicleState veh_state_msg;

  OperationModeState operation_mode_state_msg;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<VelocityReport>::SharedPtr pub_vel_report;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steer_report;

  rclcpp::Subscription<AutowareState>::SharedPtr sub_autoware_state;
  rclcpp::Subscription<VehicleState>::SharedPtr sub_veh_state;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode;

  rclcpp::Client<SetRoutePoints>::SharedPtr cli_set_route_points;
  rclcpp::Client<ChangeOperationMode>::SharedPtr cli_set_operation_mode;
  rclcpp::Client<ChangeAutowareControl>::SharedPtr cli_set_autoware_control;

  void on_timer();

  void pub_vehicle_report();

  void set_route_points();
  void set_operation_mode(uint8_t mode);
  void set_autoware_control(bool autoware_control);

  void vehStateCB(const VehicleState::SharedPtr msg);
  void autowareStateCB(const AutowareState::SharedPtr msg);
  void operationModeStateCB(const OperationModeState::SharedPtr msg);
};

}

#endif