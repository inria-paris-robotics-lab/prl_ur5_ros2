#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class WaitForJointStates : public rclcpp::Node {
public:
  WaitForJointStates() : Node("wait_for_joint_states"), received_(false) {
    RCLCPP_INFO(this->get_logger(), "Waiting for /joint_states topic...");
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&WaitForJointStates::jointStateCallback, this,
                  std::placeholders::_1));
  }

  bool received() const { return received_; }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!msg->name.empty()) {
      RCLCPP_INFO(this->get_logger(), "Received first /joint_states message.");
      received_ = true;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  bool received_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WaitForJointStates>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  while (rclcpp::ok() && !node->received()) {
    exec.spin_some();
    std::this_thread::sleep_for(50ms);
  }

  rclcpp::shutdown();
  return 0;
}
