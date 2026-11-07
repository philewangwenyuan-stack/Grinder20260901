#include <fstream>
#include <iomanip>
#include <sstream>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"

using namespace regulated_pure_pursuit_controller;
// Isolated, non-hardware fixture. Runs the production controller, while pose
// and 10 Hz odometry are supplied by an explicitly simplified body model.
class Fixture : public RegulatedPurePursuitController {
 public:
  void feedback(const nav_msgs::Odometry& odom) {
    odom_helper_.odomCallback(boost::make_shared<nav_msgs::Odometry>(odom));
    std::lock_guard<std::mutex> lock(line_odom_mutex_); line_odom_=odom;
  }
  int phase() const { return static_cast<int>(line_route_.phase); }
  bool rotating() const { return is_rotating_to_heading_; }
  bool verifyInterruptedRotationReset() {
    line_route_.phase=line_corner::Phase::Rotate;
    line_corner_rotation_=true; is_rotating_to_heading_=true;
    resetRotateToHeadingState();
    const bool ok=line_route_.phase==line_corner::Phase::Brake &&
      !line_corner_rotation_ && !is_rotating_to_heading_;
    line_route_.reacquire(); return ok;
  }
  bool reached() const { return goal_reached_; }
  double remaining(const geometry_msgs::PoseStamped& pose) {
    auto c=line_route_.current({pose.pose.position.x,pose.pose.position.y});
    return c?line_route_.remaining(*c,{pose.pose.position.x,pose.pose.position.y}):1e9;
  }
  bool cornerRotation() const { return line_corner_rotation_; }
  void supervisorStop() { line_reset_requested_.store(true); }

};
geometry_msgs::Quaternion quaternion(double yaw) {
  tf2::Quaternion q; q.setRPY(0,0,yaw); return tf2::toMsg(q);
}
int main(int argc,char** argv) {
  if (argc<5) { std::cerr<<"path.csv output.csv new_branch(0|1) model_tau_s [lateral_m] [yaw_rad]\n"; return 2; }
  ros::init(argc,argv,"line_corner_sim");
  ros::NodeHandle root_node;
  ros::Time::setNow(ros::Time(100));
  const bool enabled=std::stoi(argv[3]); const double tau=std::stod(argv[4]);
  std::vector<geometry_msgs::PoseStamped> plan;
  std::ifstream input(argv[1]); std::string row;
  std::getline(input,row);
  while (std::getline(input,row)) {
    std::replace(row.begin(),row.end(),',',' '); std::istringstream parser(row);
    geometry_msgs::PoseStamped p; p.header.frame_id="map";
    if (parser>>p.pose.position.x>>p.pose.position.y) plan.push_back(p);
  }
  if (plan.size()<2) return 3;
  // Recorded global paths contain keypoints. Match the scheduler's local
  // segment densification rather than feed long sparse edges to PathHandler.
  std::vector<geometry_msgs::PoseStamped> dense;
  for (std::size_t i=0;i+1<plan.size();++i) {
    const auto a=plan[i], b=plan[i+1];
    const double length=std::hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y);
    const int n=std::max(1,static_cast<int>(std::ceil(length/.1)));
    for (int j=0;j<n;++j) { auto p=a; const double f=double(j)/n;
      p.pose.position.x+=f*(b.pose.position.x-a.pose.position.x);
      p.pose.position.y+=f*(b.pose.position.y-a.pose.position.y);dense.push_back(p); }
  }
  dense.push_back(plan.back());plan=std::move(dense);
  for (std::size_t i=0;i<plan.size();++i) {
    auto a=plan[i?i-1:0].pose.position, b=plan[i+1<plan.size()?i+1:i].pose.position;
    plan[i].pose.orientation=quaternion(std::atan2(b.y-a.y,b.x-a.x));
  }
  auto d=plan[1].pose.position; auto first=plan.front().pose.position;
  double yaw=std::atan2(d.y-first.y,d.x-first.x);
  double offset=argc>5?std::stod(argv[5]):.06;
  double x=first.x-std::sin(yaw)*offset,y=first.y+std::cos(yaw)*offset;
  yaw+=argc>6?std::stod(argv[6]):.03;
  tf2_ros::Buffer tf;
  tf.setUsingDedicatedThread(true); // This fixture populates transforms synchronously.
  auto update_tf=[&]() {
    geometry_msgs::TransformStamped t; t.header.frame_id="map";t.child_frame_id="base_link";
    t.header.stamp=ros::Time::now();t.transform.translation.x=x;t.transform.translation.y=y;
    t.transform.rotation=quaternion(yaw);tf.setTransform(t,"offline_fixture");
  };
  update_tf();
  ros::NodeHandle private_n("~");
  private_n.setParam("costmap/global_frame","map");private_n.setParam("costmap/robot_base_frame","base_link");
  private_n.setParam("costmap/update_frequency",0.0);private_n.setParam("costmap/publish_frequency",0.0);
  private_n.setParam("costmap/width",100); private_n.setParam("costmap/height",100);
  private_n.setParam("costmap/origin_x",-50.0);private_n.setParam("costmap/origin_y",-50.0);
  private_n.setParam("costmap/resolution",.05);
  XmlRpc::XmlRpcValue plugins;plugins.setSize(0); private_n.setParam("costmap/plugins",plugins);
  costmap_2d::Costmap2DROS costmap("costmap",tf); costmap.stop();
  costmap.getCostmap()->resizeMap(2000,2000,.05,-50,-50);
  std::vector<geometry_msgs::Point> footprint(4);
  footprint[0].x=-.4;footprint[0].y=-.47; footprint[1].x=1.2;footprint[1].y=-.47;
  footprint[2].x=1.2;footprint[2].y=.47; footprint[3].x=-.4;footprint[3].y=.47;
  costmap.setUnpaddedRobotFootprint(footprint);
  ros::NodeHandle cfg("~rpp");
  cfg.setParam("use_line_based_corner_approach",enabled);
  // Identical A/B settings from the supplied RPP configuration. No calibrated
  // hardware delay is asserted; tau is an explicit sensitivity scenario.
  cfg.setParam("odom_topic","/offline/odom_wheel");
  cfg.setParam("desired_linear_vel",.03);cfg.setParam("max_linear_vel",.15);
  cfg.setParam("max_angular_vel",.10);cfg.setParam("max_linear_accel",.4);cfg.setParam("max_angular_accel",.4);
  cfg.setParam("lookahead_dist",.9);cfg.setParam("use_velocity_scaled_lookahead_dist",false);
  cfg.setParam("use_fixed_curvature_lookahead",true);cfg.setParam("curvature_lookahead_dist",.55);
  cfg.setParam("use_rotate_to_heading",true);cfg.setParam("use_corner_aware_rotate_to_heading",true);
  cfg.setParam("rotate_to_heading_angular_vel",.10);cfg.setParam("rotate_to_heading_min_angle",.45);
  cfg.setParam("rotate_to_heading_exit_angle",.08);cfg.setParam("rotate_to_heading_stable_cycles",5);
  cfg.setParam("rotate_to_heading_corner_distance",.10);
  cfg.setParam("low_speed_angular_deadband",.010);cfg.setParam("low_speed_min_angular_vel",.010);
  cfg.setParam("use_command_velocity_for_accel_limit",false);cfg.setParam("use_command_angular_velocity_for_accel_limit",false);
  cfg.setParam("use_approach_velocity_scaling",false);cfg.setParam("min_approach_linear_velocity",.01);
  cfg.setParam("approach_velocity_scaling_dist",.2);cfg.setParam("use_regulated_linear_velocity_scaling",true);
  cfg.setParam("regulated_linear_scaling_min_radius",1.2);cfg.setParam("regulated_linear_scaling_min_speed",.016);
  cfg.setParam("use_cost_regulated_linear_velocity_scaling",false);
  cfg.setParam("use_collision_detection",false);cfg.setParam("use_fixed_distance_collision_detection",false);
  cfg.setParam("goal_dist_tol",.12);cfg.setParam("angle_tol",.05);
  cfg.setParam("theta_stopped_vel",.01);cfg.setParam("trans_stopped_vel",.01);
  cfg.setParam("max_robot_pose_search_dist",2.5);
  ros::NodeHandle move_base("/move_base");move_base.setParam("controller_frequency",20.0);
  Fixture controller;controller.initialize("rpp",&tf,&costmap);controller.setPlan(plan);
  if (enabled) {
    if (!controller.verifyInterruptedRotationReset()) return 9;
    geometry_msgs::PoseStamped p; p.header.frame_id="map";p.header.stamp=ros::Time::now();
    p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation=quaternion(yaw);
    for (int fault=0;fault<4;++fault) {
      nav_msgs::Odometry o;o.header.stamp=ros::Time(100);
      if (fault==0) o.header.stamp=ros::Time(99);
      if (fault==1) o.header.stamp=ros::Time(100.1);
      if (fault==2) o.twist.twist.angular.z=std::numeric_limits<double>::quiet_NaN();
      if (fault==3) o.header.stamp=ros::Time();
      controller.feedback(o);
      geometry_msgs::TwistStamped cmd,vel;std::string message;
      if (controller.computeVelocityCommands(p,vel,cmd,message)!=mbf_msgs::ExePathResult::SUCCESS ||
          cmd.twist.linear.x!=0 || cmd.twist.angular.z!=0 ||
          message!="line_corner_waiting_for_fresh_odom") return 6;
    }
    controller.supervisorStop();
  }
  std::ofstream output(argv[2]);output<<std::setprecision(12)<<"t,x,y,yaw,v,w,cmd_v,cmd_w,phase,rotating\n";
  const double dt=.05;double v=0,w=0;
  for (int step=0;step<100000;++step) {
    const double t=step*dt;ros::Time::setNow(ros::Time(100+t));update_tf();
    if (step%2==0) { nav_msgs::Odometry odom;odom.header.stamp=ros::Time::now();
      odom.child_frame_id="base_link";odom.twist.twist.linear.x=v;odom.twist.twist.angular.z=w;controller.feedback(odom); }
    geometry_msgs::PoseStamped pose;pose.header.frame_id="map";pose.header.stamp=ros::Time::now();
    pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.orientation=quaternion(yaw);
    if (step%20==0) {
      const auto phase=controller.phase();
      controller.setPlan(plan);
      if (enabled && controller.phase()!=phase) return 7;
    }
    geometry_msgs::TwistStamped cmd,velocity;std::string message;
    auto result=controller.computeVelocityCommands(pose,velocity,cmd,message);
    if (result!=mbf_msgs::ExePathResult::SUCCESS) { std::cerr<<message<<"\n"; return 4; }
    if (enabled && controller.cornerRotation() && controller.remaining(pose)>.050001) {
      std::cerr<<"Outgoing rotation before longitudinal stop tolerance\n"; return 8;
    }
    output<<t<<','<<x<<','<<y<<','<<yaw<<','<<v<<','<<w<<','<<cmd.twist.linear.x<<','<<cmd.twist.angular.z<<','<<controller.phase()<<','<<controller.rotating()<<'\n';
    if (controller.reached()) return 0;
    const double alpha=tau<=0?1:1-std::exp(-dt/tau);
    v+=alpha*(cmd.twist.linear.x-v);w+=alpha*(cmd.twist.angular.z-w);
    x+=dt*v*std::cos(yaw+dt*w/2);y+=dt*v*std::sin(yaw+dt*w/2);yaw+=dt*w;
  }
  std::cerr<<"Simulation did not finish within 5000 s\n";return 5;
}
