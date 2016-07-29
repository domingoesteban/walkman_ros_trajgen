
// STL
#include <thread>
#include <mutex>
#include <cmath> // M_PI
#include <fstream>
#include <iomanip>      // std::setprecision

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// XSENS
#include "mvnbiomech/socket.h"
#include "mvnbiomech/parsermanager.h"


#define HUMANOID_DOF 37
#define MVNSUIT_DOF 54 //18 * 3 (Njoints*Axes)


#define deg2rad(angleDegrees) (angleDegrees * M_PI / 180.0)
#define rad2deg(angleRadians) (angleRadians * 180.0 / M_PI)


#define LOG_FORMAT " "
#define LOG_PRECISION 6


class Node {

public:
    Node();

    ~Node() {

    }

    void Run();

private:
    std::shared_ptr<ros::NodeHandle> nh_;
    double logger_rate_;

    std::shared_ptr<std::thread> mvnbiomech_server_thread_;
    void MVNBiomechServerThread();
    std::vector<double> mvnbiomech_angles_;

    void LogThread();
    std::string log_file_name;
    std::shared_ptr<std::thread> log_thread_;


    // ROS Subscribers
    ros::Subscriber joint_state_subs_;
    void JointStateCallback(const sensor_msgs::JointStateConstPtr& message );
    std::vector<double> gazebo_angles_;

};


Node::Node() {

  // Node Handler
  nh_.reset(new ros::NodeHandle("~"));


  // Resize Vectors
  mvnbiomech_angles_.resize(MVNSUIT_DOF);
  gazebo_angles_.resize(HUMANOID_DOF-6);


  // Get log rate from Parameter
  nh_->param("mvnbiomech_jointstate_logger_rate", logger_rate_, 40.0);

  // MVNStudio Thread
  mvnbiomech_server_thread_.reset(new std::thread( &Node::MVNBiomechServerThread, this));

  // Log Thread
  log_thread_.reset(new std::thread( &Node::LogThread, this));

  // Subscribe to joint_state
  joint_state_subs_ = nh_->subscribe("/bigman/joint_states", 1, &Node::JointStateCallback, this);
}

void Node::Run()  {

  ros::spin();

}


void Node::MVNBiomechServerThread() {
  Socket* socket;
  uint16_t port;

  uint bufferSize;
  char* buffer;
  int len;

  ParserManager* parserManager;
  JointAnglesDatagram* jointAngles;
  JointAngle joint;

  port = 2004;
  socket = new Socket(IP_UDP);
  socket->bind(port);

  // Buffer
  bufferSize = 500; //22*20+24=464 for jointAngles
  buffer = new char[bufferSize];

  // Parser Manager
  parserManager = new ParserManager(false, false);


  while (nh_->ok()) {
    len = socket->read(buffer, bufferSize);

    if (len > 0) {
      parserManager->readDatagram(buffer);

      // Joint Angles
      jointAngles = parserManager->getJointAnglesDatagram();

      if (jointAngles != NULL) {
        // From pelvis to T8
        joint = jointAngles->getItem(1, 2);
        mvnbiomech_angles_.at(0) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(1) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(2) = deg2rad(joint.rotation[2]);
        joint = jointAngles->getItem(2, 3);
        mvnbiomech_angles_.at(3) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(4) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(5) = deg2rad(joint.rotation[2]);
        joint = jointAngles->getItem(3, 4);
        mvnbiomech_angles_.at(6) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(7) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(8) = deg2rad(joint.rotation[2]);
        joint = jointAngles->getItem(4, 5);
        mvnbiomech_angles_.at(9) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(10) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(11) = deg2rad(joint.rotation[2]);

        // from T8 to Head:
        joint = jointAngles->getItem(5, 6);
        mvnbiomech_angles_.at(12) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(13) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(14) = deg2rad(joint.rotation[2]);
        joint = jointAngles->getItem(6, 7);
        mvnbiomech_angles_.at(15) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(16) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(17) = deg2rad(joint.rotation[2]);

        // Left Shoulder
        joint = jointAngles->getItem(12, 13);
        mvnbiomech_angles_.at(18) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(19) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(20) = deg2rad(joint.rotation[2]);
        // Right Shoulder
        joint = jointAngles->getItem(8, 9);
        mvnbiomech_angles_.at(21) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(22) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(23) = deg2rad(joint.rotation[2]);

        // Left elbow
        joint = jointAngles->getItem(13, 14);
        mvnbiomech_angles_.at(24) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(25) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(26) = deg2rad(joint.rotation[2]);
        // Right elbow
        joint = jointAngles->getItem(9, 10);
        mvnbiomech_angles_.at(27) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(28) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(29) = deg2rad(joint.rotation[2]);

        // Left wrist
        joint = jointAngles->getItem(14, 15);
        mvnbiomech_angles_.at(30) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(31) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(32) = deg2rad(joint.rotation[2]);
        // Right wrist
        joint = jointAngles->getItem(10, 11);
        mvnbiomech_angles_.at(33) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(34) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(35) = deg2rad(joint.rotation[2]);

        // Left hip
        joint = jointAngles->getItem(1, 20);
        mvnbiomech_angles_.at(36) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(37) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(38) = deg2rad(joint.rotation[2]);
        // Right hip
        joint = jointAngles->getItem(1, 16);
        mvnbiomech_angles_.at(39) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(40) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(41) = deg2rad(joint.rotation[2]);

        // Left knee
        joint = jointAngles->getItem(20, 21);
        mvnbiomech_angles_.at(42) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(43) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(44) = deg2rad(joint.rotation[2]);
        // Right knee
        joint = jointAngles->getItem(16, 17);
        mvnbiomech_angles_.at(45) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(46) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(47) = deg2rad(joint.rotation[2]);

        // Left Ankle
        joint = jointAngles->getItem(21, 22);
        mvnbiomech_angles_.at(48) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(49) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(50) = deg2rad(joint.rotation[2]);
        // Right Ankle
        joint = jointAngles->getItem(17, 18);
        mvnbiomech_angles_.at(51) = deg2rad(joint.rotation[0]);
        mvnbiomech_angles_.at(52) = deg2rad(joint.rotation[1]);
        mvnbiomech_angles_.at(53) = deg2rad(joint.rotation[2]);

      }
    }
  }

  delete parserManager;
  delete[] buffer;
  delete socket;
}


void Node::LogThread() {
  ros::Rate r(logger_rate_);

  std::ofstream log_file;
  nh_->param<std::string>("log_file_name", log_file_name, "walkman_trajectories.txt");
  log_file.open (log_file_name.c_str());

  // ROS time
  ros::Time ros_time;
  ros::Time::now();

  // Set Precision
  log_file << std::fixed;
  log_file << std::setprecision(LOG_PRECISION);

  ROS_INFO_STREAM("Logging at " << logger_rate_<< " Hz ...");

  while (nh_->ok()) {
    ros_time  = ros::Time::now();

    // Time (seconds and milliseconds)
    log_file << (ros_time.sec * 1.0) << LOG_FORMAT << (ros_time.nsec / 1000000.0);

    // MVNSuit
    for (unsigned int ii = 0; ii < mvnbiomech_angles_.size(); ii++) {
      log_file << LOG_FORMAT << mvnbiomech_angles_.at(ii);
    }

    // Gazebo JointState
    for (unsigned int ii = 0; ii < gazebo_angles_.size(); ii++) {
      log_file << LOG_FORMAT << gazebo_angles_.at(ii);
    }

    log_file << std::endl;
    r.sleep();
  }

  log_file.close();

}


void Node::JointStateCallback(const sensor_msgs::JointStateConstPtr &message) {
  //ROS_INFO_STREAM(message->position.size()) ;
  for (unsigned int ii = 0; ii < message->position.size(); ii++) {
    gazebo_angles_.at(ii) = message->position.at(ii);
  }
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "mvnbiomech_jointstate_logger");

    Node node;
    node.Run();

    return 0;
}
