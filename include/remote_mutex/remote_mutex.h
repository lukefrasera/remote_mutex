#include <string>
#include "ros/ros.h"
#include "baxter_demos/remote_mutex.h"

#define LOCK true
#define RELEASE false
class RemoteMutex {
 public:
  baxter_demos::remote_mutex msg;
  std::string topic_;
  RemoteMutex(std::string name, std::string topic) {
    msg.request.node = name;
    topic_ = topic;
  }
  RemoteMutex() {}
  bool Lock(float potential) {
    msg.request.request = LOCK;
    msg.request.activation_potential = potential;
    if (ros::service::call(topic_.c_str(), msg)) {
      return msg.response.success;
    }
  }
  bool Release() {
    msg.request.request = RELEASE;
    if (ros::service::call(topic_.c_str(), msg)) {
      return msg.response.success;
    }
  }
};