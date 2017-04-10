#ifndef PHIDGET_INTERFACE_KIT
#define PHIDGET_INTERFACE_KIT

#include <string>

#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/publisher.h>
#include <ros/rate.h>
#include <ros/time.h>
#include <ros/timer.h>

#include <phidget/basic_device.hpp>

#include <phidget_msgs/InterfaceKitState.h>

#include <phidget21.h>

namespace phidget {

class InterfaceKit : public BasicDevice< CPhidgetInterfaceKitHandle, CPhidgetInterfaceKit_create > {
public:
  InterfaceKit(ros::NodeHandle nh, const std::string &ns = "~") : Base(ns) {
    namespace rn = ros::names;
    namespace rp = ros::param;

    // load parameters
    const ros::Rate rate(rp::param(rn::append(ns, "rate"), 10.));

    // ensure the device is an interface kit
    expectTrue(getDeviceClass() == PHIDCLASS_INTERFACEKIT, "Not an interface kit");

    // start publishing state of the device
    publisher_ = nh.advertise< phidget_msgs::InterfaceKitState >("state", 1);
    timer_ = nh.createTimer(rate.expectedCycleTime(), &InterfaceKit::publishState, this);
  }

  virtual ~InterfaceKit() {}

private:
  void publishState(const ros::TimerEvent &) {
    // do not publish if no subscribers
    if (publisher_.getNumSubscribers() <= 0) {
      return;
    }

    // get state of the device
    phidget_msgs::InterfaceKitState state;
    state.header.stamp = ros::Time::now();
    {
      int count;
      expectOk(CPhidgetInterfaceKit_getInputCount(handle_, &count));
      state.inputs.resize(count);
      for (int i = 0; i < count; ++i) {
        int input;
        expectOk(CPhidgetInterfaceKit_getInputState(handle_, i, &input));
        state.inputs[i] = input;
      }
    }
    {
      int count;
      expectOk(CPhidgetInterfaceKit_getSensorCount(handle_, &count));
      state.sensor_raw_values.resize(count);
      for (int i = 0; i < count; ++i) {
        int value;
        expectOk(CPhidgetInterfaceKit_getSensorRawValue(handle_, i, &value));
        state.sensor_raw_values[i] = value;
      }
    }

    // publish the state of the device
    publisher_.publish(state);
  }

private:
  ros::Publisher publisher_;
  ros::Timer timer_;
};
}

#endif // PHIDGET_INTERFACE_KIT