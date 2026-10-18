/*
# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: May, 2023.                                                                     #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) IFRA Gazebo-ROS2 Link Attacher. URL: https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.
*/

// Physics mutations are serialized on Gazebo's update thread. The joint is
// owned here, not registered twice in Model's joint list.
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <chrono>
#include <deque>
#include <future>
#include <map>
#include <mutex>
#include "ros2_linkattacher/gazebo_link_attacher.hpp"
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

namespace gazebo_ros {
class GazeboLinkAttacherPrivate {
 public:
  struct Command {
    bool attach;
    std::string model1, link1, model2, link2;
    std::promise<std::pair<bool, std::string>> result;
    std::chrono::steady_clock::time_point deadline;
  };
  gazebo::physics::WorldPtr world;
  Node::SharedPtr node;
  gazebo::event::ConnectionPtr update;
  rclcpp::Service<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_service;
  rclcpp::Service<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_service;
  std::mutex mutex;
  std::deque<std::shared_ptr<Command>> commands;
  std::map<std::string, gazebo::physics::JointPtr> joints;
  unsigned serial = 0;

  template<class Request, class Response>
  void Submit(bool attaching, Request req, Response res) {
    auto command = std::make_shared<Command>();
    command->attach = attaching;
    command->model1=req->model1_name; command->link1=req->link1_name;
    command->model2=req->model2_name; command->link2=req->link2_name;
    command->deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    auto future=command->result.get_future();
    { std::lock_guard<std::mutex> guard(mutex); commands.push_back(command); }
    if (future.wait_for(std::chrono::seconds(4)) != std::future_status::ready) {
      res->success=false; res->message="Physics update did not process request"; return;
    }
    auto result=future.get(); res->success=result.first; res->message=result.second;
  }

  std::pair<bool,std::string> Apply(const Command &c) {
    const auto key=c.model1+"::"+c.link1+"|"+c.model2+"::"+c.link2;
    auto existing=joints.find(key);
    if (!c.attach) {
      if (existing==joints.end()) return {false,"Attachment does not exist"};
      existing->second->Detach();
      existing->second->Fini();
      joints.erase(existing);
      return {true,"Released fixed attachment"};
    }
    if (existing!=joints.end()) return {false,"Already attached"};
    if (!joints.empty()) return {false,"Gripper already carries an object"};
    auto m1=world->ModelByName(c.model1), m2=world->ModelByName(c.model2);
    if (!m1 || !m2) return {false,"Model not found"};
    auto l1=m1->GetLink(c.link1), l2=m2->GetLink(c.link2);
    if (!l1 || !l2) return {false,"Link not found"};
    if (l1->WorldPose().Pos().Distance(l2->WorldPose().Pos())>.13)
      return {false,"Object is outside physical grasp distance"};
    auto joint=world->Physics()->CreateJoint("fixed",m1);
    if (!joint) return {false,"Cannot create fixed joint"};
    joint->SetName("competition_grasp_"+std::to_string(++serial));
    joint->Load(l1,l2,ignition::math::Pose3d::Zero);
    joint->Init();
    joints.emplace(key,joint);
    return {true,"Created fixed attachment"};
  }

  void OnUpdate() {
    std::deque<std::shared_ptr<Command>> pending;
    { std::lock_guard<std::mutex> guard(mutex); pending.swap(commands); }
    for (auto &c:pending) {
      try {
        if (std::chrono::steady_clock::now()>c->deadline)
          c->result.set_value({false,"Request expired before physics update"});
        else c->result.set_value(Apply(*c));
      } catch (const std::exception &error) {
        c->result.set_value({false,error.what()});
      }
    }
  }
};

GazeboLinkAttacher::GazeboLinkAttacher():impl_(std::make_unique<GazeboLinkAttacherPrivate>()) {}
GazeboLinkAttacher::~GazeboLinkAttacher() { impl_->update.reset(); }
void GazeboLinkAttacher::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) {
  impl_->world=world; impl_->node=Node::Get(sdf);
  impl_->update=gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboLinkAttacherPrivate::OnUpdate,impl_.get()));
  impl_->attach_service=impl_->node->create_service<linkattacher_msgs::srv::AttachLink>(
    "ATTACHLINK",[this](linkattacher_msgs::srv::AttachLink::Request::SharedPtr req,
                       linkattacher_msgs::srv::AttachLink::Response::SharedPtr res) {
      impl_->Submit(true,req,res);
    });
  impl_->detach_service=impl_->node->create_service<linkattacher_msgs::srv::DetachLink>(
    "DETACHLINK",[this](linkattacher_msgs::srv::DetachLink::Request::SharedPtr req,
                       linkattacher_msgs::srv::DetachLink::Response::SharedPtr res) {
      impl_->Submit(false,req,res);
    });
}
GZ_REGISTER_WORLD_PLUGIN(GazeboLinkAttacher)
} // namespace gazebo_ros
