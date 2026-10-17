#include <array>
#include <functional>
#include <memory>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{

class JakaInitialPosePlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr) override
  {
    model_ = model;
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&JakaInitialPosePlugin::OnWorldUpdate, this));

    gzmsg << "JAKA initial pose plugin loaded for model ["
          << model_->GetName() << "]\n";
  }

private:
  void OnWorldUpdate()
  {
    if (!model_) {
      return;
    }

    // ros2_control's initial_value initializes its state interface only. Hold
    // the physical Gazebo joints briefly while the controller is loading. The
    // launch file sends the identical controller target before this handoff.
    if (model_->GetWorld()->SimTime() > common::Time(5.0)) {
      update_connection_.reset();
      gzmsg << "JAKA initial pose applied; handing joints to ros2_control\n";
      return;
    }
    static constexpr std::array<const char *, 6> joint_names = {
      "shoulder_joint", "upperArm_joint", "foreArm_joint",
      "wrist1_joint", "wrist2_joint", "wrist3_joint"};
    // (-90, 90, 0, 180, 90, 90) degrees.
    static constexpr std::array<double, 6> positions = {
      -1.5707963267948966, 1.5707963267948966, 0.0,
      3.141592653589793, 1.5707963267948966, 1.5707963267948966};

    const bool first_update = !logged_first_update_;
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      auto joint = model_->GetJoint(joint_names[i]);
      if (!joint) {
        gzerr << "JAKA initial pose: joint [" << joint_names[i]
              << "] was not found\n";
        continue;
      }
      const bool set_ok = joint->SetPosition(0, positions[i], false);
      joint->SetVelocity(0, 0.0);
      if (first_update) {
        gzmsg << "JAKA initial pose joint [" << joint_names[i]
              << "] set=" << set_ok << " position=" << joint->Position(0)
              << " target=" << positions[i] << "\n";
      }
    }
    logged_first_update_ = true;
  }

  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;
  bool logged_first_update_{false};
};

GZ_REGISTER_MODEL_PLUGIN(JakaInitialPosePlugin)

}  // namespace gazebo
