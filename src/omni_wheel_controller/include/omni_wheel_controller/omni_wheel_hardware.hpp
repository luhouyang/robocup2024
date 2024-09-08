#ifndef OMNI_WHEEL_CONTROLLER__OMNI_WHEEL_HARDWARE_HPP_
#define OMNI_WHEEL_CONTROLLER__OMNI_WHEEL_HARDWARE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <string>
#include <vector>
#include <map>  // Include the map header

namespace omni_wheel_controller
{

class RobotSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotSystem);

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;  // New member for effort state
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;  // New member for effort command

  std::vector<double> ft_states_;
  std::vector<double> ft_command_;

  std::map<std::string, std::vector<std::string>> joint_interfaces;  // Declare joint_interfaces as a map
};

}  // namespace omni_wheel_controller

#endif  // OMNI_WHEEL_CONTROLLER__OMNI_WHEEL_HARDWARE_HPP_
