#ifndef PHIDGET_BASIC_DEVICE
#define PHIDGET_BASIC_DEVICE

#include <stdexcept>

#include <ros/duration.h>
#include <ros/names.h>
#include <ros/param.h>

#include <phidget21.h>

namespace phidget {

template < typename Handle, int (*CreateFunction)(Handle *) > class BasicDevice {
protected:
  typedef BasicDevice< Handle, CreateFunction > Base;

public:
  BasicDevice(const std::string &ns = "~") {
    // load parameters
    const int serial_number(ros::param::param(ros::names::append(ns, "serial_number"), -1));
    const ros::Duration timeout(ros::param::param(ros::names::append(ns, "timeout"), 10.));

    // create and open the device
    expectOk(CreateFunction(&handle_));
    expectOk(CPhidget_open(getHandle(), serial_number));

    // ensure the device attached
    expectOk(CPhidget_waitForAttachment(getHandle(), timeout.toSec() * 1000));
  }

  virtual ~BasicDevice() {
    // finalize the device
    expectOk(CPhidget_close(getHandle()));
    expectOk(CPhidget_delete(getHandle()));
  }

  CPhidget_DeviceClass getDeviceClass() {
    CPhidget_DeviceClass ret;
    expectOk(CPhidget_getDeviceClass(getHandle(), &ret));
    return ret;
  }

protected:
  static void expectOk(const int result) {
    if (result == EPHIDGET_OK) {
      return;
    }
    const char *error;
    CPhidget_getErrorDescription(result, &error);
    throw std::runtime_error(error);
  }

  static void expectTrue(const bool result, const std::string &message) {
    if (result == true) {
      return;
    }
    throw std::runtime_error(message);
  }

private:
  CPhidgetHandle getHandle() { return reinterpret_cast< CPhidgetHandle >(handle_); }

protected:
  Handle handle_;
};
}

#endif // PHIDGET_BASIC_DEVICE