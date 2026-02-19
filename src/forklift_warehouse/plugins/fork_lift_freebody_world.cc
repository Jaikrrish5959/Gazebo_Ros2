// Standard library includes FIRST to avoid macro pollution
#include <memory>
#include <string>

// Gazebo includes
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

// ROS and SDF includes
#include <rclcpp/rclcpp.hpp>
#include <sdf/Model.hh>
#include <sdf/Root.hh>

namespace forklift_gazebo {
class ForkLiftFreeBodyWorld : public gz::sim::System,
                              public gz::sim::ISystemConfigure {
private:
  rclcpp::Node::SharedPtr ros_node_;

public:
  ForkLiftFreeBodyWorld() = default;
  ~ForkLiftFreeBodyWorld() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override {
    (void)_entity;
    (void)_sdf;
    // Initialize ROS2
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    this->ros_node_ =
        rclcpp::Node::make_shared("forklift_freebody_world_plugin");

    RCLCPP_INFO(this->ros_node_->get_logger(),
                "ForkLiftFreeBodyWorld plugin loaded");

    // Create entity creator
    gz::sim::SdfEntityCreator creator(_ecm, _eventMgr);

    // Load and spawn the forklift freebody model
    const int NUM_BOTS = 1;
    RCLCPP_INFO(this->ros_node_->get_logger(), "Total bots: %d", NUM_BOTS);

    // Spawn forklift at origin
    this->AddModel(creator, "forklift_freebody", "forklift_freebody", 0.0, 0.0,
                   0.0);
  }

private:
  void AddModel(gz::sim::SdfEntityCreator &_creator,
                const std::string &_modelName, const std::string &_instanceName,
                double _x, double _y, double _z) {
    (void)_creator;
    // Create SDF root
    sdf::Root root;

    // Try to load model from Gazebo resource path
    std::string modelPath = "model://" + _modelName;

    // For now, we'll let Gazebo handle model loading through world file
    // includes This is a simplified version - full implementation would load
    // SDF and spawn
    RCLCPP_INFO(
        this->ros_node_->get_logger(),
        "Model spawning configured for: %s at position (%.2f, %.2f, %.2f)",
        _instanceName.c_str(), _x, _y, _z);
  }
};
} // namespace forklift_gazebo

// Register the plugin
GZ_ADD_PLUGIN(forklift_gazebo::ForkLiftFreeBodyWorld, gz::sim::System,
              forklift_gazebo::ForkLiftFreeBodyWorld::ISystemConfigure)
