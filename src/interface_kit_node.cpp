#include <stdexcept>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <phidget/interface_kit.hpp>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "phidget_interface_kit_node");
  ros::NodeHandle nh;

  try {
    phidget::InterfaceKit device(nh);
    ros::spin();
  } catch (const std::runtime_error &error) {
    ROS_ERROR_STREAM(error.what());
    return 1;
  }

  return 0;
}