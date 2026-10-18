#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class AnimatedModels : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _parent;
      std::string modelName = _parent->GetName(); // 获取模型名称用于区分控制逻辑

      // 根据模型名称设置不同运动轨迹
      if (modelName == "Move1")
      {
        gazebo::common::PoseAnimationPtr anim(new gazebo::common::PoseAnimation("move1_anim", 32.0, true));
        gazebo::common::PoseKeyFrame *key;
        
        key = anim->CreateKeyFrame(0.0);
        key->Translation(ignition::math::Vector3d(-1.5, -1.5, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(8.0);
        key->Translation(ignition::math::Vector3d(-3.5, -1.5, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(16.0);
        key->Translation(ignition::math::Vector3d(-5.5, -1.5, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(24.0);
        key->Translation(ignition::math::Vector3d(-3.5, -1.5, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(32.0);
        key->Translation(ignition::math::Vector3d(-1.5, -1.5, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        _parent->SetAnimation(anim);
      }
      else if (modelName == "Move2")
      {
        gazebo::common::PoseAnimationPtr anim(new gazebo::common::PoseAnimation("move2_anim", 32.0, true));
        gazebo::common::PoseKeyFrame *key;
        key = anim->CreateKeyFrame(0.0);
        key->Translation(ignition::math::Vector3d(1.0, 3.0, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(8.0);
        key->Translation(ignition::math::Vector3d(1.0, 1.0, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(16.0);
        key->Translation(ignition::math::Vector3d(1.0, -1.0, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(24.0);
        key->Translation(ignition::math::Vector3d(1.0, 1.0, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(32.0);
        key->Translation(ignition::math::Vector3d(1.0, 3.0, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        _parent->SetAnimation(anim);
      }
    }

    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(AnimatedModels)
}
