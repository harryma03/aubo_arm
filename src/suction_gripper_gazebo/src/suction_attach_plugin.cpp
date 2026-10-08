#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
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

    // ROS service callbacks run independently of Gazebo's physics update.
    // Creating, detaching or destroying a physics joint from that callback
    // thread can race the physics engine and eventually deadlock gzserver.
    // Queue those operations and execute them at WorldUpdateBegin instead.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&SuctionAttachPlugin::OnWorldUpdate, this));

    // Fixed links can appear shortly after ModelPlugin::Load().  Try now for
    // diagnostics and retry again for every suction request.  Never silently
    // fall back to wrist3_Link: doing so shifts the measured contact point by
    // the tool length and can attach a cube while the pad is visibly far away.
    if (!ResolveSuctionLink()) {
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "Suction TCP/pad/cup is not available yet; will retry on /suction_on");
    }

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
      "Supported targets: grasp_cube and every grasp_cube_* model");
  }

private:
  enum class PhysicsCommand
  {
    kNone,
    kAttach,
    kDetach
  };

  struct CommandResult
  {
    bool success;
    std::string message;
  };

  static bool IsSupportedTargetName(const std::string & name)
  {
    return name == "grasp_cube" || name.rfind("grasp_cube_", 0) == 0;
  }

  bool ResolveSuctionLink()
  {
    if (suction_link_) {
      return true;
    }

    const std::vector<std::string> candidates = {
      "suction_tcp",
      "suction_pad_link",
      "suction_cup_link"
    };

    for (const auto & name : candidates) {
      auto link = model_->GetLink(name);
      if (!link) {
        continue;
      }
      suction_link_ = link;
      suction_link_name_ = name;
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "Using Gazebo link [%s] as suction attachment parent",
        suction_link_name_.c_str());
      return true;
    }
    return false;
  }

  void On(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    SubmitPhysicsCommand(PhysicsCommand::kAttach, response);
  }

  CommandResult AttachInPhysicsThread()
  {
    if (!ResolveSuctionLink()) {
      return {
        false,
        "Cannot find suction_tcp, suction_pad_link or suction_cup_link"
      };
    }

    // Only one object can be attached at one time.
    if (joint_) {
      return {true, "Suction already ON: " + attached_model_name_};
    }

    physics::ModelPtr nearest_model;
    physics::LinkPtr nearest_link;

    std::string nearest_name;

    double nearest_distance =
      std::numeric_limits<double>::max();

    // The suction_tcp link is the contact point. When Gazebo has lumped the
    // fixed TCP link, reconstruct that point from the selected parent link.
    // All current suction parent links place the TCP 95 mm along local +Z.
    auto suction_pose = suction_link_->WorldPose();
    auto suction_position = suction_pose.Pos();

    if (suction_link_name_ != "suction_tcp") {
      suction_position +=
        suction_pose.Rot().RotateVector(
        ignition::math::Vector3d(0.0, 0.0, 0.095));
    }

    // ---------------------------------------------------------
    // Find the single nearest cube to the suction link.
    // ---------------------------------------------------------
    for (const auto & target_model : world_->Models()) {
      if (!target_model) {
        continue;
      }

      const std::string target_name = target_model->GetName();
      if (!IsSupportedTargetName(target_name)) {
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
      return {
        false,
        "Cannot find any grasp_cube or grasp_cube_* model with cube_link"
      };
    }

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Nearest suction target: [%s], distance = %.4f m",
      nearest_name.c_str(),
      nearest_distance);

    // A suction grasp must be close to the contact point. The previous
    // 0.32 m threshold could attach a visually unrelated cube.
    constexpr double max_attach_distance = 0.10;

    if (nearest_distance > max_attach_distance) {
      return {
        false,
        "Nearest cube [" +
        nearest_name +
        "] too far: " +
        std::to_string(nearest_distance) +
        " m"
      };
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

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Suction ON: [%s] attached to [%s]",
      attached_model_name_.c_str(),
      suction_link_name_.c_str());

    return {true, attached_model_name_ + " attached"};
  }


  void Off(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    SubmitPhysicsCommand(PhysicsCommand::kDetach, response);
  }

  CommandResult DetachInPhysicsThread()
  {
    if (!joint_) {
      return {true, "Suction already OFF"};
    }

    const std::string released_name =
      attached_model_name_;

    joint_->Detach();
    joint_.reset();

    attached_model_name_.clear();

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Suction OFF: [%s] detached",
      released_name.c_str());

    return {true, released_name + " detached"};
  }

  void SubmitPhysicsCommand(
    PhysicsCommand command,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    // Serialize /suction_on and /suction_off calls.  One outstanding command
    // keeps result ownership deterministic even with a multithreaded executor.
    std::unique_lock<std::mutex> service_lock(service_mutex_);
    std::uint64_t command_id;

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (pending_command_ != PhysicsCommand::kNone) {
        response->success = false;
        response->message = "Another suction command is already pending";
        return;
      }

      command_id = ++next_command_id_;
      pending_command_id_ = command_id;
      pending_command_ = command;
    }

    std::unique_lock<std::mutex> lock(command_mutex_);
    const bool completed = command_cv_.wait_for(
      lock,
      std::chrono::seconds(2),
      [this, command_id]() {
        return completed_command_id_ >= command_id;
      });

    if (!completed) {
      // If Gazebo is paused and has not consumed the command, withdraw it so
      // a later request is not permanently rejected as "already pending".
      if (
        pending_command_id_ == command_id &&
        pending_command_ != PhysicsCommand::kNone)
      {
        pending_command_ = PhysicsCommand::kNone;
      }
      response->success = false;
      response->message =
        "Timed out waiting for Gazebo physics thread to process suction command";
      return;
    }

    response->success = completed_success_;
    response->message = completed_message_;
  }

  void OnWorldUpdate()
  {
    PhysicsCommand command;
    std::uint64_t command_id;

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (pending_command_ == PhysicsCommand::kNone) {
        return;
      }

      command = pending_command_;
      command_id = pending_command_id_;
      pending_command_ = PhysicsCommand::kNone;
    }

    CommandResult result{false, "Unknown suction command"};

    try {
      if (command == PhysicsCommand::kAttach) {
        result = AttachInPhysicsThread();
      } else if (command == PhysicsCommand::kDetach) {
        result = DetachInPhysicsThread();
      }
    } catch (const std::exception & error) {
      result.success = false;
      result.message = std::string("Gazebo suction operation failed: ") + error.what();
      RCLCPP_ERROR(ros_node_->get_logger(), "%s", result.message.c_str());
    } catch (...) {
      result.success = false;
      result.message = "Gazebo suction operation failed with an unknown exception";
      RCLCPP_ERROR(ros_node_->get_logger(), "%s", result.message.c_str());
    }

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      completed_command_id_ = command_id;
      completed_success_ = result.success;
      completed_message_ = result.message;
    }
    command_cv_.notify_all();
  }


  physics::ModelPtr model_;
  physics::WorldPtr world_;

  physics::LinkPtr suction_link_;
  physics::JointPtr joint_;

  std::string suction_link_name_;
  std::string attached_model_name_;

  event::ConnectionPtr update_connection_;

  std::mutex service_mutex_;
  std::mutex command_mutex_;
  std::condition_variable command_cv_;
  PhysicsCommand pending_command_{PhysicsCommand::kNone};
  std::uint64_t next_command_id_{0};
  std::uint64_t pending_command_id_{0};
  std::uint64_t completed_command_id_{0};
  bool completed_success_{false};
  std::string completed_message_;

  gazebo_ros::Node::SharedPtr ros_node_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    suction_on_srv_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    suction_off_srv_;
};

GZ_REGISTER_MODEL_PLUGIN(SuctionAttachPlugin)

}  // namespace gazebo
