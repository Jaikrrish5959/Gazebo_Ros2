#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/Name.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>
#include <string>

namespace forklift_gazebo {
class ForkLiftPlugin : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate {
private:
  // ROS2 node
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;

  // Gazebo entities
  gz::sim::Entity model_entity_;
  gz::sim::Entity shift_joint_;
  gz::sim::Entity lift_joint_;

  std::string model_name_;

  // Store latest joint commands
  std::map<std::string, double> joint_commands_;
  std::mutex command_mutex_;

public:
  ForkLiftPlugin() = default;
  ~ForkLiftPlugin() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager & /*_eventMgr*/) override {
    (void)_sdf;
    // Store model entity
    this->model_entity_ = _entity;

    // Get model name
    auto name_comp = _ecm.Component<gz::sim::components::Name>(_entity);
    if (name_comp) {
      this->model_name_ = name_comp->Data();
    }

    // Initialize ROS2
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    this->ros_node_ =
        rclcpp::Node::make_shared(this->model_name_ + "_forklift_plugin");

    RCLCPP_INFO(this->ros_node_->get_logger(), "%s Loaded.",
                this->model_name_.c_str());

    // Subscribe to joint states
    this->joint_state_sub_ =
        this->ros_node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ForkLiftPlugin::OnJointState, this,
                      std::placeholders::_1));

    // Find joints
    auto joints = _ecm.ChildrenByComponents(this->model_entity_,
                                            gz::sim::components::Joint());

    for (const auto &joint : joints) {
      auto joint_name_comp = _ecm.Component<gz::sim::components::Name>(joint);
      if (joint_name_comp) {
        std::string joint_name = joint_name_comp->Data();

        // Look for "shift" in joint name
        if (joint_name.find("shift") != std::string::npos) {
          this->shift_joint_ = joint;
          RCLCPP_INFO(this->ros_node_->get_logger(), "Found shift joint: %s",
                      joint_name.c_str());
        }
        // Look for "lift" in joint name
        else if (joint_name.find("lift") != std::string::npos) {
          this->lift_joint_ = joint;
          RCLCPP_INFO(this->ros_node_->get_logger(), "Found lift joint: %s",
                      joint_name.c_str());
        }
      }
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo & /*_info*/,
                 gz::sim::EntityComponentManager &_ecm) override {
    // Check if ROS2 is still running before spinning
    if (!rclcpp::ok()) {
      return;
    }

    // Spin ROS2 node
    rclcpp::spin_some(this->ros_node_);

    // Apply joint commands
    std::lock_guard<std::mutex> lock(this->command_mutex_);

    for (const auto &[joint_name, position] : this->joint_commands_) {
      gz::sim::Entity joint_entity = gz::sim::kNullEntity;

      if (joint_name == "shift") {
        joint_entity = this->shift_joint_;
      } else if (joint_name == "lift") {
        joint_entity = this->lift_joint_;
      }

      if (joint_entity != gz::sim::kNullEntity) {
        // Set joint position using JointPositionReset component
        auto pos_reset_comp =
            _ecm.Component<gz::sim::components::JointPositionReset>(
                joint_entity);
        if (!pos_reset_comp) {
          _ecm.CreateComponent(
              joint_entity,
              gz::sim::components::JointPositionReset({position}));
        } else {
          pos_reset_comp->Data()[0] = position;
        }
      }
    }
  }

private:
  void OnJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->name.empty())
      return;

    std::string joint_name = msg->name[0];
    double position = msg->position[0];

    RCLCPP_INFO(this->ros_node_->get_logger(),
                "Received joint state: %s, position: %f", joint_name.c_str(),
                position);

    std::lock_guard<std::mutex> lock(this->command_mutex_);
    this->joint_commands_[joint_name] = position;
  }
};
} // namespace forklift_gazebo

// Register the plugin
GZ_ADD_PLUGIN(forklift_gazebo::ForkLiftPlugin, gz::sim::System,
              forklift_gazebo::ForkLiftPlugin::ISystemConfigure,
              forklift_gazebo::ForkLiftPlugin::ISystemPreUpdate)
