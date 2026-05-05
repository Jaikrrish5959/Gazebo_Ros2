#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

namespace forklift_gazebo {

class ForkLiftPlugin : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate {
private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  gz::sim::Entity model_entity_;
  gz::sim::Entity shift_joint_{gz::sim::kNullEntity};
  gz::sim::Entity lift_joint_{gz::sim::kNullEntity};
  gz::sim::Entity wheel_fl_{gz::sim::kNullEntity};
  gz::sim::Entity wheel_fr_{gz::sim::kNullEntity};
  gz::sim::Entity wheel_rl_{gz::sim::kNullEntity};
  gz::sim::Entity wheel_rr_{gz::sim::kNullEntity};

  std::string model_name_;
  // Target positions for lift/shift
  double target_lift_{0.0};
  double target_shift_{0.0};
  std::mutex command_mutex_;

  // Max velocity for lift/shift joints (m/s)
  static constexpr double LIFT_SPEED  = 0.05;   // slow physical lift
  static constexpr double SHIFT_SPEED = 0.03;

  double cmd_linear_{0.0};
  double cmd_angular_{0.0};
  std::mutex vel_mutex_;

  static constexpr double WHEEL_SEPARATION = 0.26;
  static constexpr double WHEEL_RADIUS = 0.05;

  double odom_x_{0.0}, odom_y_{0.0}, odom_yaw_{0.0};
  bool first_update_{true};
  bool joints_found_{false};
  std::chrono::steady_clock::time_point last_time_;
  int odom_counter_{0};

public:
  ForkLiftPlugin() = default;
  ~ForkLiftPlugin() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override {
    this->model_entity_ = _entity;

    auto name_comp = _ecm.Component<gz::sim::components::Name>(_entity);
    if (name_comp)
      this->model_name_ = name_comp->Data();

    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    this->ros_node_ =
        rclcpp::Node::make_shared(this->model_name_ + "_forklift_plugin");

    RCLCPP_INFO(this->ros_node_->get_logger(), "%s: Plugin loaded.",
                this->model_name_.c_str());

    // Subscribers — receive target positions, move joints to them via velocity
    this->joint_state_sub_ =
        this->ros_node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
              if (msg->name.empty())
                return;
              std::lock_guard<std::mutex> lock(this->command_mutex_);
              for (size_t i = 0;
                   i < msg->name.size() && i < msg->position.size(); ++i) {
                if (msg->name[i] == "lift")
                  this->target_lift_  = msg->position[i];
                else if (msg->name[i] == "shift")
                  this->target_shift_ = msg->position[i];
              }
            });

    this->cmd_vel_sub_ =
        this->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
              std::lock_guard<std::mutex> lock(this->vel_mutex_);
              this->cmd_linear_ = msg->linear.x;
              this->cmd_angular_ = msg->angular.z;
            });

    this->odom_pub_ =
        this->ros_node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // Use Model helper to find joints by name (handles nested entities)
    gz::sim::Model model(_entity);
    this->lift_joint_ = model.JointByName(_ecm, "lift");
    this->shift_joint_ = model.JointByName(_ecm, "shift");
    this->wheel_fl_ = model.JointByName(_ecm, "wheel_front_left_joint");
    this->wheel_fr_ = model.JointByName(_ecm, "wheel_front_right_joint");
    this->wheel_rl_ = model.JointByName(_ecm, "wheel_rear_left_joint");
    this->wheel_rr_ = model.JointByName(_ecm, "wheel_rear_right_joint");

    auto logJoint = [this](const char *name, gz::sim::Entity e) {
      if (e != gz::sim::kNullEntity)
        RCLCPP_INFO(this->ros_node_->get_logger(), "  Found: %s", name);
      else
        RCLCPP_WARN(this->ros_node_->get_logger(), "  Missing: %s", name);
    };
    logJoint("lift", this->lift_joint_);
    logJoint("shift", this->shift_joint_);
    logJoint("wheel_front_left_joint", this->wheel_fl_);
    logJoint("wheel_front_right_joint", this->wheel_fr_);
    logJoint("wheel_rear_left_joint", this->wheel_rl_);
    logJoint("wheel_rear_right_joint", this->wheel_rr_);
  }

  void PreUpdate(const gz::sim::UpdateInfo &,
                 gz::sim::EntityComponentManager &_ecm) override {
    if (!rclcpp::ok())
      return;
    rclcpp::spin_some(this->ros_node_);

    // If joints not found yet, try again (they may appear after first sim step)
    if (!this->joints_found_) {
      gz::sim::Model model(this->model_entity_);
      if (this->lift_joint_ == gz::sim::kNullEntity)
        this->lift_joint_ = model.JointByName(_ecm, "lift");
      if (this->shift_joint_ == gz::sim::kNullEntity)
        this->shift_joint_ = model.JointByName(_ecm, "shift");
      if (this->wheel_fl_ == gz::sim::kNullEntity)
        this->wheel_fl_ = model.JointByName(_ecm, "wheel_front_left_joint");
      if (this->wheel_fr_ == gz::sim::kNullEntity)
        this->wheel_fr_ = model.JointByName(_ecm, "wheel_front_right_joint");
      if (this->wheel_rl_ == gz::sim::kNullEntity)
        this->wheel_rl_ = model.JointByName(_ecm, "wheel_rear_left_joint");
      if (this->wheel_rr_ == gz::sim::kNullEntity)
        this->wheel_rr_ = model.JointByName(_ecm, "wheel_rear_right_joint");

      if (this->wheel_fl_ != gz::sim::kNullEntity) {
        this->joints_found_ = true;
        RCLCPP_INFO(this->ros_node_->get_logger(), "All wheel joints found!");
      }
    }

    // --- LIFT/SHIFT: velocity-based position control ---
    // This moves joints physically (respects collisions) instead of teleporting.
    {
      std::lock_guard<std::mutex> lock(this->command_mutex_);

      auto driveJointToTarget = [&](
          gz::sim::Entity je, double target, double speed) {
        if (je == gz::sim::kNullEntity) return;

        // Read current joint position
        auto pos_comp = _ecm.Component<gz::sim::components::JointPosition>(je);
        if (!pos_comp || pos_comp->Data().empty()) {
          // Ensure position component exists so we can read it next tick
          _ecm.CreateComponent(je, gz::sim::components::JointPosition({0.0}));
          return;
        }
        double current = pos_comp->Data()[0];
        double error   = target - current;

        // Dead-zone: stop if close enough
        double vel = 0.0;
        if (std::abs(error) > 0.002)
          vel = (error > 0 ? 1.0 : -1.0) * speed;

        auto vc = _ecm.Component<gz::sim::components::JointVelocityCmd>(je);
        if (!vc)
          _ecm.CreateComponent(je, gz::sim::components::JointVelocityCmd({vel}));
        else
          vc->Data()[0] = vel;
      };

      driveJointToTarget(this->lift_joint_,  this->target_lift_,  LIFT_SPEED);
      driveJointToTarget(this->shift_joint_, this->target_shift_, SHIFT_SPEED);
    }

    // --- WHEEL VELOCITY ---
    double linear, angular;
    {
      std::lock_guard<std::mutex> lock(this->vel_mutex_);
      linear = this->cmd_linear_;
      angular = this->cmd_angular_;
    }

    double v_left = (linear - angular * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS;
    double v_right = (linear + angular * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS;

    auto setVel = [&](gz::sim::Entity w, double v) {
      if (w == gz::sim::kNullEntity)
        return;
      auto c = _ecm.Component<gz::sim::components::JointVelocityCmd>(w);
      if (!c)
        _ecm.CreateComponent(w, gz::sim::components::JointVelocityCmd({v}));
      else
        c->Data()[0] = v;
    };
    setVel(this->wheel_fl_, v_left);
    setVel(this->wheel_fr_, v_right);
    setVel(this->wheel_rl_, v_left);
    setVel(this->wheel_rr_, v_right);

    // --- ODOMETRY ---
    auto now = std::chrono::steady_clock::now();
    if (this->first_update_) {
      this->last_time_ = now;
      this->first_update_ = false;
      return;
    }
    double dt = std::chrono::duration<double>(now - this->last_time_).count();
    this->last_time_ = now;

    if (dt > 0.0 && dt < 1.0) {
      this->odom_x_ += linear * std::cos(this->odom_yaw_) * dt;
      this->odom_y_ += linear * std::sin(this->odom_yaw_) * dt;
      this->odom_yaw_ += angular * dt;
    }

    this->odom_counter_++;
    if (this->odom_counter_ >= 100) {
      this->odom_counter_ = 0;
      nav_msgs::msg::Odometry msg;
      msg.header.stamp = this->ros_node_->get_clock()->now();
      msg.header.frame_id = "odom";
      msg.child_frame_id = "base_link";
      msg.pose.pose.position.x = this->odom_x_;
      msg.pose.pose.position.y = this->odom_y_;
      msg.pose.pose.orientation.w = std::cos(this->odom_yaw_ / 2.0);
      msg.pose.pose.orientation.z = std::sin(this->odom_yaw_ / 2.0);
      msg.twist.twist.linear.x = linear;
      msg.twist.twist.angular.z = angular;
      this->odom_pub_->publish(msg);
    }
  }
};

} // namespace forklift_gazebo

GZ_ADD_PLUGIN(forklift_gazebo::ForkLiftPlugin, gz::sim::System,
              forklift_gazebo::ForkLiftPlugin::ISystemConfigure,
              forklift_gazebo::ForkLiftPlugin::ISystemPreUpdate)
