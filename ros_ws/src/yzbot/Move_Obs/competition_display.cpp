// Project-local Gazebo GUI render-rate control and measured FPS telemetry.
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/GuiEvents.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/rendering/ogre_gazebo.h>
#include <QTimer>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <algorithm>
namespace gazebo {
class CompetitionDisplay : public GUIPlugin {
 public:
  CompetitionDisplay() { this->setFixedSize(0,0); }
  void Load(sdf::ElementPtr sdf) override {
    target_=sdf->Get<double>("fps",60.0).first;
    target_=std::clamp(target_,1.0,240.0);
    timer_=new QTimer(this);
    QObject::connect(timer_,&QTimer::timeout,this,[this](){Update();});
    timer_->start(500);
  }
 private:
  void Update() {
    auto camera=gui::get_active_camera();
    if (!camera || !camera->Initialized())return;
    if (camera->RenderRate()!=target_)gui::Events::setRenderRate(target_);
    const auto path=std::getenv("YZBOT_LOG");
    if (!path)return;
    auto system=Ogre::Root::getSingleton().getRenderSystem();
    const auto &caps=system->getCapabilities();
    std::ofstream output(std::string(path)+"/gazebo-render.json");
    output<<"{\"target_fps\":"<<target_<<",\"configured_fps\":"<<camera->RenderRate()
          <<",\"average_fps\":"<<camera->AvgFPS()<<",\"renderer\":\""<<caps->getDeviceName()<<"\"}";
  }
  QTimer *timer_=nullptr;
  double target_=60.;
};
GZ_REGISTER_GUI_PLUGIN(CompetitionDisplay)
}
