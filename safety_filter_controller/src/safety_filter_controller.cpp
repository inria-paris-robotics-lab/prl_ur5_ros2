#include "safety_filter_controller/safety_filter_controller.hpp"

#include <set>
#include <sstream>

#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/collision/broadphase-callbacks.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/string.hpp" // AJOUTÉ POUR LE TOPIC ROBOT_DESCRIPTION

namespace safety_filter_controller {

using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;

SafetyFilterController::SafetyFilterController()
    : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn SafetyFilterController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("joints.active_joint",
                                           std::vector<std::string>());
    auto_declare<std::vector<std::string>>("joints.passive_joint",
                                           std::vector<std::string>());
    auto_declare<std::string>("srdf_filename", "");
    auto_declare<std::vector<double>>("max_delta_per_joint",
                                      std::vector<double>());
    auto_declare<int>("horizon_steps", 5);
    auto_declare<double>("control_period_estimate", 0.008);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SafetyFilterController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  joint_names_ =
      get_node()->get_parameter("joints.active_joint").as_string_array();
  const std::vector<std::string> passive_joint_names =
      get_node()->get_parameter("joints.passive_joint").as_string_array();
  max_delta_per_joint_ =
      get_node()->get_parameter("max_delta_per_joint").as_double_array();
  horizon_steps_ =
      static_cast<int>(get_node()->get_parameter("horizon_steps").as_int());
  control_period_estimate_ =
      get_node()->get_parameter("control_period_estimate").as_double();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "'joints.active_joint' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (max_delta_per_joint_.size() != joint_names_.size()) {
    max_delta_per_joint_.assign(joint_names_.size(), 0.05);
  }

  // ---- RÉCUPÉRATION DE L'URDF DEPUIS LE TOPIC robot_description ----
  const std::string &robot_description = get_robot_description();

  if (robot_description.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_description is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // ---- BUILD PINOCCHIO MODEL ----
  try {
    pinocchio::urdf::buildModelFromXML(robot_description, model_);
    data_ = pinocchio::Data(model_);

    std::istringstream xml_stream(robot_description);
    pinocchio::urdf::buildGeom(model_, xml_stream, pinocchio::COLLISION,
                               geom_model_, std::vector<std::string>{});

    geom_model_.addAllCollisionPairs();

    // ---- Retrait des paires "toujours en collision" (links adjacents, ex.
    // shoulder_pan<->shoulder_lift qui se touchent au niveau du joint) ----
    // Sans ce filtrage, addAllCollisionPairs() garde ces paires et le check
    // renvoie "collision" en permanence, dès la toute première évaluation,
    // même sans commande. C'est le même mécanisme que le SRDF de MoveIt
    // (<disable_collisions>).
    const std::string srdf_filename =
        get_node()->get_parameter("srdf_filename").as_string();

    if (!srdf_filename.empty()) {
      const size_t n_pairs_before = geom_model_.collisionPairs.size();

      pinocchio::srdf::removeCollisionPairs(model_, geom_model_, srdf_filename,
                                            false);

      RCLCPP_INFO(
          get_node()->get_logger(),
          "SRDF '%s' chargé : %zu paires de collision retirées (%zu -> %zu)",
          srdf_filename.c_str(),
          n_pairs_before - geom_model_.collisionPairs.size(), n_pairs_before,
          geom_model_.collisionPairs.size());
    } else {
      RCLCPP_WARN(
          get_node()->get_logger(),
          "Aucun 'srdf_filename' fourni : les paires de collision entre links "
          "adjacents (toujours en contact) ne sont PAS filtrées, le check de "
          "collision risque de renvoyer 'collision' en permanence, même au "
          "repos.");
    }

    geom_data_ = pinocchio::GeometryData(geom_model_);

    broadphase_manager_ =
        std::make_unique<BroadPhaseManager>(&model_, &geom_model_, &geom_data_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to build Pinocchio model: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // ------------------------------------------------------------------------
  // Résolution par nom, dans le modèle Pinocchio, d'une liste de joints.
  // Remplit idx_q / idx_v (taille = names.size()) et vérifie que chaque
  // joint existe et est simple (nq=nv=1, revolute/prismatic).
  // ------------------------------------------------------------------------
  auto resolve_joints = [this](const std::vector<std::string> &names,
                               std::vector<int> &idx_q, std::vector<int> &idx_v,
                               const char *param_label) -> bool {
    idx_q.resize(names.size());
    idx_v.resize(names.size());

    for (size_t i = 0; i < names.size(); ++i) {
      if (!model_.existJointName(names[i])) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "joint '%s' (%s) absent du modèle URDF", names[i].c_str(),
                     param_label);
        return false;
      }

      const auto joint_id = model_.getJointId(names[i]);
      const auto &joint = model_.joints[joint_id];

      if (joint.nq() != 1 || joint.nv() != 1) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "joint '%s' (%s) a nq=%d/nv=%d : seuls les joints simples "
                     "(revolute/prismatic, nq=nv=1) sont supportés",
                     names[i].c_str(), param_label, joint.nq(), joint.nv());
        return false;
      }

      idx_q[i] = joint.idx_q();
      idx_v[i] = joint.idx_v();
    }
    return true;
  };

  // ------------------------------------------------------------------------
  // Table "commande" : joints ACTIFS (joints.active_joint), ordre =
  // joint_names_ = q_cmd_ros_ = q_current_ros_ = command_interfaces_.
  // ------------------------------------------------------------------------
  if (!resolve_joints(joint_names_, cmd_idx_q_, cmd_idx_v_,
                      "joints.active_joint")) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // ------------------------------------------------------------------------
  // Table "état" : ACTIFS + PASSIFS explicitement listés dans le yaml (et
  // non plus dérivés automatiquement de tout le modèle URDF, qui pourrait
  // contenir des joints sans state interface réelle côté hardware, ex.
  // outillage, mimic joints...). L'ordre de state_joint_names_ définit
  // l'ordre attendu des state interfaces, voir state_interface_configuration().
  // ------------------------------------------------------------------------
  state_joint_names_.clear();
  state_joint_names_.reserve(joint_names_.size() + passive_joint_names.size());
  state_joint_names_.insert(state_joint_names_.end(), joint_names_.begin(),
                            joint_names_.end());
  state_joint_names_.insert(state_joint_names_.end(),
                            passive_joint_names.begin(),
                            passive_joint_names.end());

  {
    std::set<std::string> seen;
    for (const auto &name : state_joint_names_) {
      if (!seen.insert(name).second) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "joint '%s' listé à la fois (ou en double) dans "
                     "joints.active_joint / joints.passive_joint",
                     name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
  }

  if (!resolve_joints(state_joint_names_, state_idx_q_, state_idx_v_,
                      "joints.active/passive_joint")) {
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto n_ros = static_cast<Eigen::Index>(joint_names_.size());
  q_current_ros_ = Eigen::VectorXd::Zero(n_ros);
  q_cmd_ros_ = Eigen::VectorXd::Zero(n_ros);
  last_safe_command_ros_ = Eigen::VectorXd::Zero(n_ros);

  q_pin_ = pinocchio::neutral(model_);
  v_pin_ = Eigen::VectorXd::Zero(model_.nv);

  has_safe_command_ = false;

  // "joints.active_joint" (donc l'ordre de q_cmd_ros_) est fixé une fois pour
  // toutes ici, à la configuration : pas besoin de transporter les noms à
  // chaque message, Float64MultiArray dans l'ordre de joint_names_ suffit et
  // reste léger.
  ref_subscriber_ =
      get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
          "~/commands", rclcpp::SystemDefaultsQoS(),
          [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            input_ref_buffer_.writeFromNonRT(msg);
          });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SafetyFilterController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  read_current_state();
  q_cmd_ros_ = q_current_ros_;
  last_safe_command_ros_ = q_current_ros_;
  has_safe_command_ = true;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SafetyFilterController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  has_safe_command_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration
SafetyFilterController::command_interface_configuration() const {
  InterfaceConfiguration conf;
  conf.type = interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto &joint : joint_names_) {
    conf.names.push_back(joint + "/position");
  }
  return conf;
}

InterfaceConfiguration
SafetyFilterController::state_interface_configuration() const {
  InterfaceConfiguration conf;
  conf.type = interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(state_joint_names_.size() * 2);

  // IMPORTANT : cet ordre doit rester synchronisé avec
  // state_idx_q_/state_idx_v_ (construits dans on_configure() à partir de la
  // même liste state_joint_names_).
  for (const auto &joint_name : state_joint_names_) {
    conf.names.push_back(joint_name + "/position");
    conf.names.push_back(joint_name + "/velocity");
  }

  return conf;
}

void SafetyFilterController::read_current_state() {
  // Reconstruit la configuration COMPLETE du robot (tous les joints du
  // modèle, y compris ceux non commandés par ce contrôleur, ex: 2e bras).
  q_pin_ = pinocchio::neutral(model_);

  for (size_t i = 0; i < state_joint_names_.size(); ++i) {
    q_pin_[state_idx_q_[i]] = state_interfaces_[2 * i].get_value();
  }

  // Extrait la position actuelle des seuls joints commandés (pour le
  // delta-check et le calcul de vitesse de l'horizon).
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q_current_ros_[static_cast<Eigen::Index>(i)] = q_pin_[cmd_idx_q_[i]];
  }
}

bool SafetyFilterController::check_delta_limits(
    const Eigen::VectorXd &q_from, const Eigen::VectorXd &q_to) const {
  for (Eigen::Index i = 0; i < q_from.size(); ++i) {
    if (std::abs(q_to[i] - q_from[i]) >
        max_delta_per_joint_[static_cast<size_t>(i)]) {
      return false;
    }
  }
  return true;
}

bool SafetyFilterController::check_horizon_collision_free(
    const Eigen::VectorXd &q_cmd_ros, double dt) {
  if (dt <= 0.0) {
    dt = control_period_estimate_;
  }

  // q_pin_ a déjà été rempli par read_current_state() avec la configuration
  // COMPLETE et réelle du robot (tous les joints, commandés ou non).
  // On ne fait avancer que les joints commandés ; les autres restent figés
  // à leur position actuelle (vitesse nulle) pendant toute la simulation.
  v_pin_.setZero();

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    v_pin_[cmd_idx_v_[i]] =
        (q_cmd_ros[static_cast<Eigen::Index>(i)] - q_pin_[cmd_idx_q_[i]]) / dt;
  }

  // Copie de travail : ne pas modifier q_pin_ ici, il représente l'état réel
  // courant et doit rester intact pour le prochain appel à update().
  Eigen::VectorXd q_step = q_pin_;

  // --------------------------------------------------------------------------
  // Collision horizon
  // --------------------------------------------------------------------------

  for (int k = 0; k < horizon_steps_; ++k) {
    q_step = pinocchio::integrate(model_, q_step, v_pin_ * dt);

    pinocchio::forwardKinematics(model_, data_, q_step);

    pinocchio::updateGeometryPlacements(model_, data_, geom_model_, geom_data_);

    broadphase_manager_->update(&geom_data_);

    pinocchio::CollisionCallBackDefault callback(geom_model_, geom_data_, true);

    broadphase_manager_->collide(&callback);

    if (callback.collision) {
      // Construit la liste des paires de géométries réellement en collision,
      // à partir de geom_data_.collisionResults (aligné avec
      // geom_model_.collisionPairs), plutôt que de dumper toute la config.
      std::ostringstream oss;
      oss << "Collision detected at horizon step " << k << ": ";

      bool first_pair = true;

      for (size_t cp_idx = 0; cp_idx < geom_model_.collisionPairs.size();
           ++cp_idx) {
        if (!geom_data_.collisionResults[cp_idx].isCollision()) {
          continue;
        }

        const auto &cp = geom_model_.collisionPairs[cp_idx];
        const auto &go1 = geom_model_.geometryObjects[cp.first];
        const auto &go2 = geom_model_.geometryObjects[cp.second];

        if (!first_pair) {
          oss << " | ";
        }
        first_pair = false;

        oss << go1.name << " <-> " << go2.name;
      }

      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                           1000, "%s", oss.str().c_str());

      return false;
    }
  }

  return true;
}

controller_interface::return_type
SafetyFilterController::update(const rclcpp::Time & /*time*/,
                               const rclcpp::Duration &period) {
  read_current_state();

  auto msg = input_ref_buffer_.readFromRT();

  bool command_changed = false;
  constexpr double epsilon = 1e-6;

  if (msg && *msg && (*msg)->data.size() == joint_names_.size()) {
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const double r = (*msg)->data[i];
      const Eigen::Index idx = static_cast<Eigen::Index>(i);

      if (!std::isnan(r)) {
        if (std::abs(r - q_cmd_ros_[idx]) > epsilon) {
          q_cmd_ros_[idx] = r;
          command_changed = true;
        }
      } else {
        // NaN = maintenir la position actuelle
        if (std::abs(q_current_ros_[idx] - q_cmd_ros_[idx]) > epsilon) {
          q_cmd_ros_[idx] = q_current_ros_[idx];
          command_changed = true;
        }
      }
    }
  }

  // Pas de nouvelle commande différente :
  // inutile de refaire le calcul de sécurité.

  const double dt = period.seconds();

  bool ok = has_safe_command_;

  if (ok) {
    ok = check_delta_limits(last_safe_command_ros_, q_cmd_ros_) &&
         check_horizon_collision_free(q_cmd_ros_, dt);
  }

  if (ok) {
    last_safe_command_ros_ = q_cmd_ros_;
    has_safe_command_ = true;
  } else {
    if (command_changed) {
      RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Unsafe command detected, reverting to last safe command");
    }

    q_cmd_ros_ = has_safe_command_ ? last_safe_command_ros_ : q_current_ros_;
  }

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    (void)command_interfaces_[i].set_value(
        q_cmd_ros_[static_cast<Eigen::Index>(i)]);
  }

  return controller_interface::return_type::OK;
}

} // namespace safety_filter_controller

PLUGINLIB_EXPORT_CLASS(safety_filter_controller::SafetyFilterController,
                       controller_interface::ControllerInterface)
