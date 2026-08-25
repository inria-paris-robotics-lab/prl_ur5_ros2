#ifndef SAFETY_FILTER_CONTROLLER__SAFETY_FILTER_CONTROLLER_HPP_
#define SAFETY_FILTER_CONTROLLER__SAFETY_FILTER_CONTROLLER_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Dense>

#include "coal/broadphase/broadphase_dynamic_AABB_tree.h"
#include "pinocchio/collision/broadphase-manager.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/multibody/model.hpp"

namespace safety_filter_controller {

using BroadPhaseManager =
    pinocchio::BroadPhaseManagerTpl<coal::DynamicAABBTreeCollisionManager>;

class SafetyFilterController
    : public controller_interface::ControllerInterface {
public:
  SafetyFilterController();

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // ---- parameters ----
  std::vector<std::string> joint_names_;
  std::vector<double> max_delta_per_joint_;
  int horizon_steps_{5};
  double control_period_estimate_{0.008};

  // ---- pinocchio model / collision ----
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::GeometryModel geom_model_;
  pinocchio::GeometryData geom_data_;
  std::unique_ptr<BroadPhaseManager> broadphase_manager_;

  // ---- index mapping : joints ACTIFS (param "joints.active_joint") ----
  // Résolus par nom dans le modèle Pinocchio à on_configure(). Même ordre
  // que joint_names_ / q_cmd_ros_ / q_current_ros_ / command_interfaces_.
  std::vector<int> cmd_idx_q_;
  std::vector<int> cmd_idx_v_;

  // ---- index mapping : ACTIFS + PASSIFS (état complet suivi) ----
  // Union explicite de joints.active_joint et joints.passive_joint (yaml),
  // PAS dérivée automatiquement de tout le modèle URDF : évite de réclamer
  // une state interface pour un joint du modèle qui n'en expose pas
  // réellement côté hardware (outillage, mimic joint, etc.).
  // state_joint_names_ fixe l'ordre utilisé à la fois par
  // state_interface_configuration() et par read_current_state() : les
  // state interfaces sont fournies par ros2_control dans cet ordre précis.
  std::vector<std::string> state_joint_names_;
  std::vector<int> state_idx_q_;
  std::vector<int> state_idx_v_;

  // ---- pre-allocated RT buffers ----
  Eigen::VectorXd q_current_ros_; // position actuelle des joints COMMANDÉS
  Eigen::VectorXd q_cmd_ros_;     // commande désirée des joints COMMANDÉS
  Eigen::VectorXd last_safe_command_ros_;
  Eigen::VectorXd q_pin_; // config Pinocchio COMPLETE (tout le modèle)
  Eigen::VectorXd v_pin_; // vitesse Pinocchio COMPLETE (tout le modèle)

  bool has_safe_command_{false};

  // ---- topic input ----
  realtime_tools::RealtimeBuffer<
      std::shared_ptr<std_msgs::msg::Float64MultiArray>>
      input_ref_buffer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      ref_subscriber_;

  // ---- helpers ----
  bool new_command_received_ = false;
  void read_current_state();
  bool check_delta_limits(const Eigen::VectorXd &q_from,
                          const Eigen::VectorXd &q_to) const;
  bool check_horizon_collision_free(const Eigen::VectorXd &q_cmd_ros,
                                    double dt);
};

} // namespace safety_filter_controller

#endif // SAFETY_FILTER_CONTROLLER__SAFETY_FILTER_CONTROLLER_HPP_
