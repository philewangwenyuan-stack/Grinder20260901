
#include <csignal>
#include <ros/ros.h>
#include "lio/super_lio_location.h"


void SigHandle(int sig) {
  LI2Sup::g_flag_run = false;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "lio");
  signal(SIGINT, SigHandle);
  ros::NodeHandle nh;
  LI2Sup::LoadParamFromRos(nh);
  LI2Sup::SuperLIOReLoc lio;
  lio.init();
  lio.run();
  ros::spin();
  return 0;
}
