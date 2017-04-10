#include <stdexcept>

#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>

#include <phidget/interface_kit.hpp>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "phidget_interface_kit_node");
  ros::NodeHandle nh;

  while (ros::ok()) {
    try {
      phidget::InterfaceKit device(nh);
      ros::spin();
    } catch (const std::runtime_error &error) {
      ROS_ERROR_STREAM(error.what());
      ros::Duration(1.).sleep();
    }
  }

  return 0;
}