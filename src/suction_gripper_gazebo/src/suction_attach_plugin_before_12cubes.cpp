#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace gazebo
{

class SuctionAttachPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();
    ros_node_ = gazebo_ros::Node::Get(sdf);

    // Gazebo may lump fixed links.
    // Try ideal suction links first, then fallback links.
    const std::vector<std::string> candidates = {
      "suction_pad_link",
      "suction_cup_link",
      "ee_link",
      "wrist3_Link"
    };

    for (const auto & name : candidates) {
      auto link = model_->GetLink(name);

      if (link) {
        suction_link_ = link;
        suction_link_name_ = name;
        break;
      }
    }

    if (!suction_link_) {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Cannot find any usable suction parent link.");

      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Gazebo links available in model:");

      for (const auto & link : model_->GetLinks()) {
        RCLCPP_ERROR(
          ros_node_->get_logger(),
          "  %s",
          link->GetName().c_str());
      }

      return;
    }

    RCLCPP_WARN(
      ros_node_->get_logger(),
      "Using Gazebo link [%s] as suction attachment parent",
      suction_link_name_.c_str());

    suction_on_srv_ =
      ros_node_->create_service<std_srvs::srv::Trigger>(
      "/suction_on",
      std::bind(
        &SuctionAttachPlugin::On,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    suction_off_srv_ =
      ros_node_->create_service<std_srvs::srv::Trigger>(
      "/suction_off",
      std::bind(
        &SuctionAttachPlugin::Off,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Suction plugin ready: /suction_on /suction_off");

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Supported targets: grasp_cube_1 ... grasp_cube_4");
  }

private:
  void On(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // Only one object can be attached at one time.
    if (joint_) {
      response->success = true;

      response->message =
        "Suction already ON: " + attached_model_name_;

      return;
    }

    // Four sequential test objects.
    // Keep grasp_cube as fallback for the old single-cube test.
    const std::vector<std::string> target_names = {
      "grasp_cube_1",
      "grasp_cube_2",
      "grasp_cube_3",
      "grasp_cube_4",
      "grasp_cube"
    };

    physics::ModelPtr nearest_model;
    physics::LinkPtr nearest_link;

    std::string nearest_name;

    double nearest_distance =
      std::numeric_limits<double>::max();

    const auto suction_position =
      suction_link_->WorldPose().Pos();

    // ---------------------------------------------------------
    // Find the single nearest cube to the suction link.
    // ---------------------------------------------------------
    for (const auto & target_name : target_names) {

      auto target_model =
        world_->ModelByName(target_name);

      if (!target_model) {
        continue;
      }

      auto target_link =
        target_model->GetLink("cube_link");

      if (!target_link) {
        RCLCPP_WARN(
          ros_node_->get_logger(),
          "Model [%s] exists but cube_link was not found",
          target_name.c_str());

        continue;
      }

      const auto target_position =
        target_link->WorldPose().Pos();

      const double distance =
        suction_position.Distance(target_position);

      RCLCPP_INFO(
        ros_node_->get_logger(),
        "Candidate [%s]: distance = %.4f m",
        target_name.c_str(),
        distance);

      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_model = target_model;
        nearest_link = target_link;
        nearest_name = target_name;
      }
    }

    // ---------------------------------------------------------
    // Nothing found.
    // ---------------------------------------------------------
    if (!nearest_model || !nearest_link) {
      response->success = false;

      response->message =
        "Cannot find any grasp_cube_1 ... grasp_cube_4";

      return;
    }

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Nearest suction target: [%s], distance = %.4f m",
      nearest_name.c_str(),
      nearest_distance);

    // Keep the threshold that already worked in the previous
    // single-cube simulation because Gazebo may use wrist3_Link
    // after fixed-joint lumping.
    constexpr double max_attach_distance = 0.25;

    if (nearest_distance > max_attach_distance) {

      response->success = false;

      response->message =
        "Nearest cube [" +
        nearest_name +
        "] too far: " +
        std::to_string(nearest_distance) +
        " m";

      return;
    }

    // ---------------------------------------------------------
    // Attach ONLY the nearest object.
    // ---------------------------------------------------------
    joint_ =
      world_->Physics()->CreateJoint(
      "fixed",
      model_);

    joint_->Attach(
      suction_link_,
      nearest_link);

    joint_->Load(
      suction_link_,
      nearest_link,
      ignition::math::Pose3d());

    joint_->Init();

    attached_model_name_ = nearest_name;

    response->success = true;

    response->message =
      attached_model_name_ + " attached";

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Suction ON: [%s] attached to [%s]",
      attached_model_name_.c_str(),
      suction_link_name_.c_str());
  }


  void Off(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!joint_) {
      response->success = true;
      response->message = "Suction already OFF";
      return;
    }

    const std::string released_name =
      attached_model_name_;

    joint_->Detach();
    joint_.reset();

    attached_model_name_.clear();

    response->success = true;

    response->message =
      released_name + " detached";

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Suction OFF: [%s] detached",
      released_name.c_str());
  }


  physics::ModelPtr model_;
  physics::WorldPtr world_;

  physics::LinkPtr suction_link_;
  physics::JointPtr joint_;

  std::string suction_link_name_;
  std::string attached_model_name_;

  gazebo_ros::Node::SharedPtr ros_node_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    suction_on_srv_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    suction_off_srv_;
};

GZ_REGISTER_MODEL_PLUGIN(SuctionAttachPlugin)

}  // namespace gazebo
