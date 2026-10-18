#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>
#include <cmath>
#include <functional>

namespace gazebo {
class CompetitionPatrol : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    start_ = sdf->Get<ignition::math::Vector3d>("start");
    end_ = sdf->Get<ignition::math::Vector3d>("end");
    period_ = sdf->HasElement("period") ? sdf->Get<double>("period") : 28.0;
    // A moving obstacle is kinematic. Keep all wheel/body transforms together;
    // do not mix Model::SetAnimation with a multi-link dynamic model.
    model_->SetStatic(true);
    connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&CompetitionPatrol::Update, this, std::placeholders::_1));
  }
 private:
  void Update(const common::UpdateInfo &info) {
    const double phase = 2.0 * M_PI * info.simTime.Double() / period_;
    const double u = 0.5 * (1.0 - std::cos(phase));
    const auto point = start_ + (end_ - start_) * u;
    const double yaw = std::atan2(end_.Y()-start_.Y(), end_.X()-start_.X());
    model_->SetWorldPose(ignition::math::Pose3d(point, ignition::math::Quaterniond(0,0,yaw)));
  }
  physics::ModelPtr model_;
  event::ConnectionPtr connection_;
  ignition::math::Vector3d start_, end_;
  double period_ = 28.0;
};
GZ_REGISTER_MODEL_PLUGIN(CompetitionPatrol)
}
