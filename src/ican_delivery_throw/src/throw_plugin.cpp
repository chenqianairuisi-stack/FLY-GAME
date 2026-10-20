// Extends release with ONE horizontal impulse. Never sets a pose or guides
// the package after release; gravity and contact are handled by Gazebo.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <map>
#include <mutex>
#include <sstream>
#include <set>

namespace delivery {
class ThrowActuator : public gazebo::ModelPlugin {
  struct Shot {
    std::string token, color, ack;
    double target_x{}, target_y{}, requested{}, launched{}, last_sample{-1.};
    bool released{false}, landed{false};
    std::ostringstream samples;
    int count{0};
  };
  gazebo::physics::WorldPtr world;
  gazebo_ros::Node::SharedPtr node;
  gazebo::event::ConnectionPtr update;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr drop, state;
  std::map<std::string, Shot> shots;
  std::mutex mutex;
  double last_status{-1.};
  void publish(const std::string &value) {
    std_msgs::msg::String m; m.data=value; state->publish(m);
  }
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    world=model->GetWorld(); node=gazebo_ros::Node::Get(sdf);
    drop=node->create_publisher<std_msgs::msg::String>("/delivery/drop_cmd",10);
    state=node->create_publisher<std_msgs::msg::String>("/delivery/throw_state",10);
    sub=node->create_subscription<std_msgs::msg::String>("/delivery/throw_cmd",10,
      [this](std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> guard(mutex);
        if (m->data=="cancel") {
          for(auto it=shots.begin();it!=shots.end();)
            if(!it->second.released) it=shots.erase(it); else ++it;
          return;
        }
        std::istringstream in(m->data); std::string token,color,extra; double target_x,target_y;
        if(!(in>>token>>color>>target_x>>target_y) || (in>>extra) ||
          !std::isfinite(target_x)||!std::isfinite(target_y) ||
          token.find_first_not_of("0123456789_abcdefghijklmnopqrstuvwxyz")!=std::string::npos ||
          !std::set<std::string>{"red","yellow","black","white"}.count(color)) return;
        if(shots.count(token)) {
          if(!shots.at(token).ack.empty()) publish(shots.at(token).ack);
          return;
        }
        for(auto &entry:shots) if(entry.second.color==color) return;
        auto &s=shots[token]; s.token=token; s.color=color;
        s.target_x=target_x; s.target_y=target_y;
        s.requested=world->SimTime().Double();
        std_msgs::msg::String command; command.data=color; drop->publish(command);
      });
    update=gazebo::event::Events::ConnectWorldUpdateBegin([this](const gazebo::common::UpdateInfo &){ tick(); });
  }
  void tick() {
    std::lock_guard<std::mutex> guard(mutex);
    const double now=world->SimTime().Double();
    if(now-last_status>1.) {publish("{\"status\":\"ready\"}");last_status=now;}
    for(auto &entry:shots) {
      auto &s=entry.second;
      auto model=world->ModelByName("package_"+s.color);
      if(!model || s.landed) continue;
      auto link=model->GetLink(); if(!link) continue;
      if(!s.released) {
        // Gravity becomes active on the official release update. Apply the
        // impulse immediately; periodic package_state must not delay it.
        if(!link->GetGravityMode()) continue;
        auto velocity=model->WorldLinearVel();
        const auto position=model->WorldPose().Pos();
        if (position.Z() < .12) continue;
        constexpr double gravity=9.8, ground_center=.025;
        const double height=position.Z()-ground_center;
        const double vz=velocity.Z();
        const double flight=(vz + std::sqrt(std::max(0.0, vz*vz + 2.*gravity*height)))/gravity;
        const double t=std::max(.05, flight);
        velocity.X((s.target_x-position.X())/t);
        velocity.Y((s.target_y-position.Y())/t);
        if (std::hypot(velocity.X(), velocity.Y()) > 2.) continue;
        model->SetLinearVel(velocity);
        s.released=true;s.launched=now;
        auto p=model->WorldPose().Pos();
        std::ostringstream out; out.precision(10);
        out<<"{\"status\":\"released\",\"token\":\""<<s.token
          <<"\",\"color\":\""<<s.color<<"\",\"sim_time\":"<<now
          <<",\"target\":["<<s.target_x<<","<<s.target_y<<"]"
          <<",\"position\":["<<p.X()<<","<<p.Y()<<","<<p.Z()
          <<"],\"velocity\":["<<velocity.X()<<","<<velocity.Y()<<","<<velocity.Z()<<"]}";
        s.ack=out.str();publish(s.ack);
      }
      auto p=model->WorldPose().Pos();
      if(now-s.last_sample>=.01) {
        if(s.count++) s.samples<<",";
        s.samples<<"["<<now-s.launched<<","<<p.X()<<","<<p.Y()<<","<<p.Z()<<"]";
        s.last_sample=now;
      }
      if(now-s.launched>.25 && p.Z()<.028 && model->WorldLinearVel().Length()<.03) {
        s.landed=true;
        // Always include the actual settled pose, even between sampling ticks.
        if(now > s.last_sample) {
          if(s.count++) s.samples<<",";
          s.samples<<"["<<now-s.launched<<","<<p.X()<<","<<p.Y()<<","<<p.Z()<<"]";
        }
        std::ostringstream out;
        out<<"{\"status\":\"landed\",\"token\":\""<<s.token<<"\",\"color\":\""
          <<s.color<<"\",\"samples\":["<<s.samples.str()<<"]}";
        publish(out.str());
      }
    }
  }
};
GZ_REGISTER_MODEL_PLUGIN(ThrowActuator)
}
