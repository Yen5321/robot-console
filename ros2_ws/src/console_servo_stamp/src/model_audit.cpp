#include <fstream>
#include <iostream>
#include <sstream>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rclcpp/rclcpp.hpp>

std::string read(const char* path) { std::ifstream f(path); if (!f) throw std::runtime_error(path); std::stringstream s; s<<f.rdbuf(); return s.str(); }
int main(int argc,char** argv) {
  if(argc!=9) {std::cerr<<"model_audit URDF SRDF q1 q2 q3 q4 q5 q6 (rad)\n";return 2;}
  rclcpp::init(argc,argv);
  auto u=urdf::parseURDF(read(argv[1])); auto s=std::make_shared<srdf::Model>(); s->initString(*u,read(argv[2]));
  auto model=std::make_shared<moveit::core::RobotModel>(u,s); planning_scene::PlanningScene scene(model);
  auto& state=scene.getCurrentStateNonConst(); std::vector<double> q;for(int i=3;i<9;++i)q.push_back(std::stod(argv[i]));
  state.setJointGroupPositions("arm",q);state.update();
  collision_detection::DistanceRequest req;collision_detection::DistanceResult res;
  req.type=collision_detection::DistanceRequestType::ALL;req.acm=&scene.getAllowedCollisionMatrix();req.group_name="arm";req.enableGroup(model);req.distance_threshold=.05;
  scene.getCollisionEnv()->distanceSelf(req,res,state);
  for(const auto& pair:res.distances)for(const auto& d:pair.second)std::cout<<pair.first.first<<" "<<pair.first.second<<" "<<d.distance<<" m\n";
  std::cout<<"minimum="<<res.minimum_distance.distance<<" collision="<<res.collision<<"\n";
  rclcpp::shutdown();return res.collision?1:0;
}
