#include "omni_wheel_controller/omni_wheel_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp" 

#include <string>
#include <vector>

namespace omni_wheel_controller
{
hardware_interface::CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // // Explicitly check and log the omni_wheel_names parameter
  // if (info.hardware_parameters.find("omni_wheel_names") != info.hardware_parameters.end())
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "omni_wheel_names: %s", info.hardware_parameters.at("omni_wheel_names").c_str());
  // }
  // else
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("RobotSystem"), "omni_wheel_names parameter not found!");
  //   return hardware_interface::CallbackReturn::ERROR;
  // }

  // Initialize vectors for 4 omni wheels
  joint_position_.resize(4, 0.0);
  joint_velocity_.resize(4, 0.0);
  joint_effort_.resize(4, 0.0);  // Add effort state
  joint_position_command_.resize(4, 0.0);
  joint_velocity_command_.resize(4, 0.0);
  joint_effort_command_.resize(4, 0.0);  // Add effort command

  // Initialize force/torque sensor readings
  ft_states_.resize(6, 0.0);
  ft_command_.resize(6, 0.0);

  // Parse joint interfaces from hardware info
  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Add joint state interfaces
  for (size_t i = 0; i < 4; ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_interfaces["position"][i], hardware_interface::HW_IF_POSITION, &joint_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_interfaces["velocity"][i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_interfaces["effort"][i], hardware_interface::HW_IF_EFFORT, &joint_effort_[i]));
  }

  // Add force/torque sensor state interfaces
  state_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_states_[0]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_states_[1]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_states_[2]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_states_[3]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_states_[4]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_states_[5]);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Add joint command interfaces
  for (size_t i = 0; i < 4; ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_interfaces["position"][i], hardware_interface::HW_IF_POSITION, &joint_position_command_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_interfaces["velocity"][i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_command_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_interfaces["effort"][i], hardware_interface::HW_IF_EFFORT, &joint_effort_command_[i]));
  }

  // Add force/torque sensor command interfaces
  command_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_command_[0]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_command_[1]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_command_[2]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_command_[3]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_command_[4]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_command_[5]);

  return command_interfaces;
}

hardware_interface::CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize joint states
  for (size_t i = 0; i < 4; ++i)
  {
    joint_position_[i] = 0.0;
    joint_velocity_[i] = 0.0;
    joint_effort_[i] = 0.0;
    joint_position_command_[i] = 0.0;
    joint_velocity_command_[i] = 0.0;
    joint_effort_command_[i] = 0.0;
  }

  // Initialize force/torque sensor states
  for (size_t i = 0; i < 6; ++i)
  {
    ft_states_[i] = 0.0;
    ft_command_[i] = 0.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Read joint states from hardware (simulated here)
  for (size_t i = 0; i < 4; ++i)
  {
    joint_position_[i] += joint_velocity_[i] * period.seconds();  // Simulate movement
    joint_velocity_[i] = joint_velocity_command_[i];  // Update velocity
    joint_effort_[i] = joint_effort_command_[i];  // Update effort
  }

  // Read force/torque sensor data (simulated here)
  for (size_t i = 0; i < 6; ++i)
  {
    ft_states_[i] = ft_command_[i];  // Simulate sensor reading
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Write joint commands to hardware (simulated here)
  for (size_t i = 0; i < 4; ++i)
  {
    joint_velocity_[i] = joint_velocity_command_[i];
    joint_position_[i] = joint_position_command_[i];
    joint_effort_[i] = joint_effort_command_[i];
  }

  // Write force/torque commands (if applicable)
  for (size_t i = 0; i < 6; ++i)
  {
    ft_states_[i] = ft_command_[i];
  }

  return hardware_interface::return_type::OK;
}

}  // namespace omni_wheel_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  omni_wheel_controller::RobotSystem, hardware_interface::SystemInterface)
