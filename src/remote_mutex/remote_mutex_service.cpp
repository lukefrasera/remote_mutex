#include <string>
#include "ros/ros.h"
#include "ros_remote_mutex.h"
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <fstream>
class RemoteMutexService;

void Record(RemoteMutexService* mut);

class RemoteMutexService {
 public:
  bool locked;
  std::string owner;
  ros::ServiceServer service;  
  ros::NodeHandle ns;
  float activation_potential;
  boost::mutex mut;
  boost::thread* record_thread;
  std::ofstream file;

  RemoteMutexService(const char* name) {
    locked = false;
    owner = "";
    activation_potential = 0.0f;
    service = ns.advertiseService(name, &RemoteMutexService::MutexRequest, this);
    record_thread = new boost::thread(&Record, this);
    file.open("/home/luke/Documents/ws/Data/remote_mutex.csv");
    file.precision(15);
  }

  ~RemoteMutexService() {
    file.close();
  }

 void RecordToFile() {
  boost::posix_time::ptime time_t_epoch(boost::gregorian::date(1970,1,1));
  boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - time_t_epoch;
  double seconds = (double)diff.total_seconds() + (double)diff.fractional_seconds() / 1000000.0;
  file  << std::fixed
        << seconds
        << ", "
        << owner
        << "\n";
        file.flush();
}

  bool MutexRequest(baxter_demos::remote_mutex::Request &req,
      baxter_demos::remote_mutex::Response &res) {
    if (req.request) {
      if (locked) {
        res.success = false;
        ROS_INFO("Mutex Already Locked - Denied Access: %s", req.node.c_str());
      } else {
        // check if Nodes activation is the highest for a second
        if (activation_potential < req.activation_potential) {
          mut.lock();
          activation_potential = req.activation_potential;
          mut.unlock();
          boost::this_thread::sleep(boost::posix_time::millisec(500));
          if (activation_potential > req.activation_potential) {
            ROS_INFO("Not Highest Activation Potential - Denied Access");
            res.success = false;
          } else {
            mut.lock();
              locked = true;
              owner = req.node;
            mut.unlock();
              res.success = true;
              ROS_INFO("Mutex Locked - Granted Access: %s", req.node.c_str());
          }
        } else {
          ROS_INFO("Not Highest Activation Potential - Denied Access");
          res.success = false;
        }
      }
    } else {
      if (locked) {
        if (req.node == owner) {
          mut.lock();
          locked = false;
          owner = "";
          activation_potential = 0.0f;
          mut.unlock();
          res.success = true;
          ROS_INFO("Mutex Unlocked - Granted Access: %s", req.node.c_str());
        } else {
          res.success = false;
          ROS_INFO("Mutex Locked - Denied Access: %s", req.node.c_str());
        }
      } else {
        res.success = false;
        ROS_INFO("Mutex Already Unlocked: %s", req.node.c_str());
      }
    }
    return true;
  }
};

void Record(RemoteMutexService* mut) {
  while (true) {
    mut->RecordToFile();
    boost::this_thread::sleep(boost::posix_time::millisec(500));
  }
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "remote_mutex_server");

  RemoteMutexService mutex(argv[1]);
  ros::AsyncSpinner spinner(8);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}