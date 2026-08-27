#include <wsg_50_interface/wsg_ros_interface.hpp>
namespace wsg_50_interface
{

  WSG50HardwareInterface::WSG50HardwareInterface(): wsg_(), last_goal_position_(0.0),mode_(0){}

  WSG50HardwareInterface::~WSG50HardwareInterface(){}

  hardware_interface::CallbackReturn WSG50HardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
  {
    // Check if the hardware interface is valid
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Getting the gripper parameters
    wsg_.name_ = info.joints[0].name; 
    wsg_.ip_ = info.hardware_parameters.at("ip_address");
    wsg_.port_ = std::stoi(info.hardware_parameters.at("port"));
    wsg_.protocol_ = info.hardware_parameters.at("protocol");
    wsg_.rate_ = std::stod(info.hardware_parameters.at("rate"));
    wsg_.grasping_force_ = std::stod(info.hardware_parameters.at("grasping_force"));
    wsg_.goal_speed_ = std::stod(info.hardware_parameters.at("speed"));
    wsg_.finger_sensors_ = (info.hardware_parameters.at("finger_sensors") == "true");

    // Check the communication protocol
    if (wsg_.protocol_ == "udp")
    {
      wsg_.local_port_ = std::stoi(info.hardware_parameters.at("local_port"));
      RCLCPP_INFO(rclcpp::get_logger("WSG50HardwareInterface"), "\nGripper joint name: %s  \nGripper IP: %s  \nGripper port: %d \nGripper local port: %d \nGripper Protocol: UDP \nGripper rate: %f  \nGripper grasping force: %f\nFingers sensor: %s", wsg_.name_.c_str(), wsg_.ip_.c_str(), wsg_.port_, wsg_.local_port_, wsg_.rate_, wsg_.grasping_force_, wsg_.finger_sensors_ ? "Enabled" : "Disabled");
    }
    else
    {
      if(wsg_.protocol_ != "tcp")
      {
        RCLCPP_ERROR(rclcpp::get_logger("WSG50HardwareInterface"), "Invalid protocol specified. Use 'tcp' or 'udp'.");
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_INFO(rclcpp::get_logger("WSG50HardwareInterface"), "\nGripper joint name: %s  \nGripper IP: %s  \nGripper port: %d \nGripper Protocol: TCP \nGripper rate: %f  \nGripper grasping force: %f\nFingers sensor: %s", wsg_.name_.c_str(), wsg_.ip_.c_str(), wsg_.port_, wsg_.rate_, wsg_.grasping_force_, wsg_.finger_sensors_ ? "Enabled" : "Disabled");
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn WSG50HardwareInterface::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("WSG50HardwareInterface"), "Activating WSG50");

    try {
      if (!wsg_.connect()) {
        RCLCPP_ERROR(rclcpp::get_logger("WSG50HardwareInterface"), "Failed to connect to WSG-50");
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (!wsg_.setup()) {
        RCLCPP_ERROR(rclcpp::get_logger("WSG50HardwareInterface"), "Failed to setup WSG-50");
        return hardware_interface::CallbackReturn::ERROR;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("WSG50HardwareInterface"), "Exception in on_activate: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> WSG50HardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(wsg_.name_, hardware_interface::HW_IF_POSITION, &wsg_.width_);
    state_interfaces.emplace_back(wsg_.name_, hardware_interface::HW_IF_VELOCITY, &wsg_.speed_);
    state_interfaces.emplace_back(wsg_.name_, hardware_interface::HW_IF_EFFORT, &wsg_.force_);
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> WSG50HardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(wsg_.name_, hardware_interface::HW_IF_POSITION, &wsg_.goal_width_);
    return command_interfaces;
  }

  hardware_interface::return_type WSG50HardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    // Read made by the thread
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type WSG50HardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    // Non-blocking: the TCP writes and the open/close logic live in
    // WSG50Driver::write_thread, off this RT loop.
    wsg_.request(wsg_.goal_width_, wsg_.goal_speed_);
    return hardware_interface::return_type::OK;
  }
  hardware_interface::CallbackReturn WSG50HardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("WSG50HardwareInterface"), "Deactivating WSG50");
    try {
      if (!wsg_.disconnect()) {
        RCLCPP_ERROR(rclcpp::get_logger("WSG50HardwareInterface"), "Failed to disconnect from WSG-50");
        return hardware_interface::CallbackReturn::ERROR;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("WSG50HardwareInterface"), "Exception in on_deactivate: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  #include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(wsg_50_interface::WSG50HardwareInterface, hardware_interface::SystemInterface)

}// namespace wsg_50_interface
