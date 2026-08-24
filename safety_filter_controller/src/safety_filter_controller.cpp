#include "safety_filter_controller/safety_filter_controller.hpp"

#include <sstream>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/collision/broadphase-callbacks.hpp"

#include "std_msgs/msg/string.hpp" // AJOUTÉ POUR LE TOPIC ROBOT_DESCRIPTION
#include "pluginlib/class_list_macros.hpp"

namespace safety_filter_controller
{

using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;

SafetyFilterController::SafetyFilterController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn SafetyFilterController::on_init()
{
  try
  {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<double>>("max_delta_per_joint", std::vector<double>());
    auto_declare<int>("horizon_steps", 5);
    auto_declare<double>("control_period_estimate", 0.008);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SafetyFilterController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  max_delta_per_joint_ = get_node()->get_parameter("max_delta_per_joint").as_double_array();
  horizon_steps_ = static_cast<int>(get_node()->get_parameter("horizon_steps").as_int());
  control_period_estimate_ = get_node()->get_parameter("control_period_estimate").as_double();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  if (max_delta_per_joint_.size() != joint_names_.size())
  {
    max_delta_per_joint_.assign(joint_names_.size(), 0.05);
  }

  // ---- RÉCUPÉRATION DE L'URDF DEPUIS LE TOPIC robot_description ----
  const std::string & robot_description = get_robot_description();

  if (robot_description.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "robot_description is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // ---- BUILD PINOCCHIO MODEL ----
  try
  {
    pinocchio::urdf::buildModelFromXML(robot_description, model_);
    data_ = pinocchio::Data(model_);

    std::istringstream xml_stream(robot_description);
    pinocchio::urdf::buildGeom(
      model_, xml_stream, pinocchio::COLLISION, geom_model_,
      std::vector<std::string>{});
      
    geom_model_.addAllCollisionPairs();
    geom_data_ = pinocchio::GeometryData(geom_model_);

    broadphase_manager_ =
      std::make_unique<BroadPhaseManager>(&model_, &geom_model_, &geom_data_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Failed to build Pinocchio model: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // ---- Map ROS joints to Pinocchio joints ----
  ros_q_to_pin_q_.resize(model_.nq);
  ros_v_to_pin_v_.resize(model_.nv);
  pin_q_to_ros_q_.assign(model_.nq, -1);

  for (size_t ros_i = 0; ros_i < model_.nq; ++ros_i)
  {
    const auto joint_id = model_.getJointId(model_.names[ros_i + 1]); // +1 to skip the universe joint

    const auto & joint =
      model_.joints[joint_id];

    ros_q_to_pin_q_[ros_i] = joint.idx_q();
    ros_v_to_pin_v_[ros_i] = joint.idx_v();

    // Mapping inverse
    for (int j = 0; j < joint.nq(); ++j)
    {
      pin_q_to_ros_q_[joint.idx_q() + j] =
        static_cast<int>(ros_i);
    }
  }

  // Store active joint index 
  for (pinocchio::JointIndex pin_i = 0; pin_i < model_.nq; ++pin_i)
  {
    const int ros_i = pin_q_to_ros_q_[pin_i];
    pin_q_is_active_.push_back(ros_i >= 0);
  }

  const auto n_ros = static_cast<Eigen::Index>(joint_names_.size());
  q_current_ros_ = Eigen::VectorXd::Zero(n_ros);
  q_cmd_ros_ = Eigen::VectorXd::Zero(n_ros);
  last_safe_command_ros_ = Eigen::VectorXd::Zero(n_ros);
  
  q_pin_ = pinocchio::neutral(model_);
  v_pin_ = Eigen::VectorXd::Zero(model_.nv); 

  has_safe_command_ = false;

  ref_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      input_ref_buffer_.writeFromNonRT(msg);
    });

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn SafetyFilterController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  read_current_state();
  q_cmd_ros_ = q_current_ros_;
  last_safe_command_ros_ = q_current_ros_;
  has_safe_command_ = true;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SafetyFilterController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  has_safe_command_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration SafetyFilterController::command_interface_configuration() const
{
  InterfaceConfiguration conf;
  conf.type = interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto & joint : joint_names_)
  {
    conf.names.push_back(joint + "/position");
  }
  return conf;
}

InterfaceConfiguration SafetyFilterController::state_interface_configuration() const
{
  InterfaceConfiguration conf;
  conf.type = interface_configuration_type::INDIVIDUAL;

  for (pinocchio::JointIndex joint_id = 1;
       joint_id < static_cast<pinocchio::JointIndex>(model_.njoints);
       ++joint_id)
  {
    const auto & joint = model_.joints[joint_id];

    // Les joints fixes n'ont pas de state interface
    if (joint.nq() == 0 || joint.nv() == 0)
    {
      continue;
    }

    const std::string & joint_name = model_.names[joint_id];

    conf.names.push_back(joint_name + "/position");
    conf.names.push_back(joint_name + "/velocity");
  }

  return conf;
}

void SafetyFilterController::read_current_state()
{
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    q_current_ros_[static_cast<Eigen::Index>(i)] = state_interfaces_[2 * i].get_value();
  }
}

bool SafetyFilterController::check_delta_limits(
  const Eigen::VectorXd & q_from, const Eigen::VectorXd & q_to) const
{
  for (Eigen::Index i = 0; i < q_from.size(); ++i)
  {
    if (std::abs(q_to[i] - q_from[i]) > max_delta_per_joint_[static_cast<size_t>(i)])
    {
      return false;
    }
  }
  return true;
}

bool SafetyFilterController::check_horizon_collision_free(
  const Eigen::VectorXd & q_current_ros,
  const Eigen::VectorXd & q_cmd_ros,
  double dt)
{
  if (dt <= 0.0)
  {
    dt = control_period_estimate_;
  }


  q_pin_ = pinocchio::neutral(model_);
  v_pin_.setZero();

  for (int pin_i = 0; pin_i < model_.nq; ++pin_i)
  {
    const int ros_i = pin_q_to_ros_q_[pin_i];

    if (ros_i >= 0)
    {
      q_pin_[pin_i] = q_current_ros[ros_i];

      v_pin_[pin_i] =
        (q_cmd_ros[ros_i] - q_current_ros[ros_i]) / dt;
    }
  }

  // --------------------------------------------------------------------------
  // Collision horizon
  // --------------------------------------------------------------------------

  for (int k = 0; k < horizon_steps_; ++k)
  {
    q_pin_ =
      pinocchio::integrate(
        model_,
        q_pin_,
        v_pin_ * dt);

    pinocchio::forwardKinematics(
      model_,
      data_,
      q_pin_);

    pinocchio::updateGeometryPlacements(
      model_,
      data_,
      geom_model_,
      geom_data_);

    broadphase_manager_->update(
      &geom_data_);

    pinocchio::CollisionCallBackDefault callback(
      geom_model_,
      geom_data_,
      true);

    broadphase_manager_->collide(
      &callback);

    if (callback.collision)
    {
      RCLCPP_INFO_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "===== Pinocchio configuration =====");

      for (
        pinocchio::JointIndex joint_id = 1;
        joint_id <
          static_cast<pinocchio::JointIndex>(model_.njoints);
        ++joint_id)
      {
        const auto & joint =
          model_.joints[joint_id];

        if (joint.nq() == 0)
        {
          continue;
        }

        std::ostringstream oss;

        oss << "Joint ["
            << joint_id
            << "] "
            << model_.names[joint_id]
            << " q=[";

        for (int j = 0; j < joint.nq(); ++j)
        {
          if (j > 0)
          {
            oss << ", ";
          }

          oss <<
            q_pin_[joint.idx_q() + j];
        }

        oss << "]";

        RCLCPP_INFO(
          get_node()->get_logger(),
          "%s",
          oss.str().c_str());
      }

      return false;
    }
  }

  return true;
}

controller_interface::return_type SafetyFilterController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  read_current_state();

  auto msg = input_ref_buffer_.readFromRT();

  bool command_changed = false;
  constexpr double epsilon = 1e-6;

  if (msg && *msg && (*msg)->data.size() == joint_names_.size())
  {
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      const double r = (*msg)->data[i];

      if (!std::isnan(r))
      {
        const Eigen::Index idx = static_cast<Eigen::Index>(i);

        if (std::abs(r - q_cmd_ros_[idx]) > epsilon)
        {
          q_cmd_ros_[idx] = r;
          command_changed = true;
        }
      }
      else
      {
        // NaN = maintenir la position actuelle
        if (std::abs(q_current_ros_[i] - q_cmd_ros_[i]) > epsilon)
        {
          q_cmd_ros_[i] = q_current_ros_[i];
          command_changed = true;
        }
      }
    }
  }

  // Pas de nouvelle commande différente :
  // inutile de refaire le calcul de sécurité.
  
  const double dt = period.seconds();

  bool ok = has_safe_command_;

  if (ok)
  {
    ok =
      check_delta_limits(last_safe_command_ros_, q_cmd_ros_) &&
      check_horizon_collision_free(q_current_ros_, q_cmd_ros_, dt);
  }

  if (ok)
  {
    last_safe_command_ros_ = q_cmd_ros_;
    has_safe_command_ = true;
  }
  else
  {
    if (command_changed)
    {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Unsafe command detected, reverting to last safe command");
    }

    q_cmd_ros_ =
      has_safe_command_ ? last_safe_command_ros_ : q_current_ros_;
  }

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    (void)command_interfaces_[i].set_value(
      q_cmd_ros_[static_cast<Eigen::Index>(i)]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace safety_filter_controller

PLUGINLIB_EXPORT_CLASS(
  safety_filter_controller::SafetyFilterController,
  controller_interface::ControllerInterface)