/*
    Joint ID:   0   |   Name:               x   |   Type:  PRISMATIC |   Actuation: 0
    Joint ID:   1   |   Name:               y   |   Type:  PRISMATIC |   Actuation: 0
    Joint ID:   2   |   Name:               z   |   Type:  PRISMATIC |   Actuation: 0
    Joint ID:   3   |   Name:               r   |   Type:   REVOLUTE |   Actuation: 0
    Joint ID:   4   |   Name:               p   |   Type:   REVOLUTE |   Actuation: 0
    Joint ID:   5   |   Name:               y   |   Type:   REVOLUTE |   Actuation: 0
    Joint ID:   6   |   Name:         LHipLat   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:   7   |   Name:         LHipYaw   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:   8   |   Name:         LHipSag   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:   9   |   Name:        LKneeSag   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  10   |   Name:         LAnkSag   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  11   |   Name:         LAnkLat   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  12   |   Name:         RHipLat   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  13   |   Name:         RHipYaw   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  14   |   Name:         RHipSag   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  15   |   Name:        RKneeSag   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  16   |   Name:         RAnkSag   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  17   |   Name:         RAnkLat   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  18   |   Name:        WaistLat   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  19   |   Name:        WaistSag   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  20   |   Name:        WaistYaw   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  21   |   Name:          LShSag   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  22   |   Name:          LShLat   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  23   |   Name:          LShYaw   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  24   |   Name:           LElbj   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  25   |   Name:   LForearmPlate   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  26   |   Name:           LWrj1   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  27   |   Name:           LWrj2   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  28   |   Name:        NeckYawj   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  29   |   Name:      NeckPitchj   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  30   |   Name:          RShSag   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  31   |   Name:          RShLat   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  32   |   Name:          RShYaw   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  33   |   Name:           RElbj   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  34   |   Name:   RForearmPlate   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  35   |   Name:           RWrj1   |   Type:   REVOLUTE |   Actuation: 1
    Joint ID:  36   |   Name:           RWrj2   |   Type:   REVOLUTE |   Actuation: 1
 *
 */


// STL
#include <queue>
#include <thread>
#include <cmath> // M_PI
#include <memory> //shared_ptr
#include <functional> // bind

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float64.h>

#include "walkman_ros_trajgen/JointLinearInterpolation.h"
#include "walkman_ros_trajgen/WBJointLinearInterpolation.h"
#include "walkman_ros_trajgen/PoseInterpolation.h"
#include "walkman_ros_trajgen/JointLinearTrajectory.h"
#include "walkman_ros_trajgen/SetPose.h"
#include "walkman_ros_trajgen/PreRecordedTrajectory.h"

#define HUMANOID_DOF 37


#define deg2rad(angleDegrees) (angleDegrees * M_PI / 180.0)
#define rad2deg(angleRadians) (angleRadians * 180.0 / M_PI)

float PI = 3.1415926536;

//enum class InterpolationType { linear, splite, polynomial5 };
enum class PoseName{TPose, NPose, APose, FPose, BPose};
enum class PreRecordedTrajectoryName{OnlyRShLat_0_90_0};

class WBTrajectoryGenerator {
public:
    WBTrajectoryGenerator();
    ~WBTrajectoryGenerator();

    int getWBJoints();
};

int WBTrajectoryGenerator::getWBJoints() {
  return 0;
}


class Node {

public:
    Node();

    ~Node() {

    }

    void Run();

private:
    std::shared_ptr<ros::NodeHandle> nh_;
    //ros::NodeHandle nh_;
    double node_rate_;
    std::vector<std_msgs::Float64> msg_vector_;
    std::vector<ros::Publisher> publisher_vector_;
    std::vector<std::queue<float>> trajectory_vector_;

    // ROS Services
    ros::CallbackQueue service_queue_;
    std::shared_ptr<std::thread> callback_queue_thread_;
    ros::ServiceServer joint_lin_interp_service_;
    ros::ServiceServer wb_joint_lin_interp_service_;
    ros::ServiceServer set_pose_service_;
    ros::ServiceServer pre_recorded_traj_service_;
    ros::ServiceServer joint_lin_traj_service_;
    ros::ServiceServer pose_interp_service_;


    int PublishWBTrajectory();
    int AssignTopicName();
    int GetLastPoint();
    int InitializeTrajectory();

    int JointLinearInterpolation(size_t joint_id, double final_point, unsigned int n_steps);
    int JointLinearTrajectory(size_t joint_id, std::vector<double>& point_array, std::vector<unsigned int>& n_steps);
    int WBJointLinearInterpolation(std::vector<size_t>& joint_id, std::vector<double>& final_point, std::vector<unsigned int>& n_steps);
    int SetPose(PoseName pose_name, unsigned int n_steps, bool clean_prev_traj);
    int PreRecordedTrajectory(PreRecordedTrajectoryName trajectory_name);
    int CleanAllTrajectories();
    int CleanJointTrajectory(size_t joint_id);
    int RepeatJointPosition(size_t joint_id, unsigned int n_times);
    int RepeatWBJointPosition(unsigned int n_times);

    void ServiceQueueThread();
    void AdvertiseServices();

    bool JointLinearInterpolationSrv(walkman_ros_trajgen::JointLinearInterpolation::Request &req,
                                    walkman_ros_trajgen::JointLinearInterpolation::Response &res);

    bool WBJointLinearInterpolationSrv(walkman_ros_trajgen::WBJointLinearInterpolation::Request &req,
                                       walkman_ros_trajgen::WBJointLinearInterpolation::Response &res);

    bool SetPoseSrv(walkman_ros_trajgen::SetPose::Request &req,
                   walkman_ros_trajgen::SetPose::Response &res);

    bool PreRecordedTrajSrv(walkman_ros_trajgen::PreRecordedTrajectory::Request &req,
                            walkman_ros_trajgen::PreRecordedTrajectory::Response &res);

    bool JointLinearTrajectorySrv(walkman_ros_trajgen::JointLinearTrajectory::Request &req,
                                  walkman_ros_trajgen::JointLinearTrajectory::Response &res);

    bool PoseInterpolationSrv(walkman_ros_trajgen::PoseInterpolation::Request &req,
                              walkman_ros_trajgen::PoseInterpolation::Response &res);
};


Node::Node() {

  // Node Handler
  nh_.reset(new ros::NodeHandle("~"));

  // Resize
  msg_vector_.resize(HUMANOID_DOF);
  publisher_vector_.resize(HUMANOID_DOF);
  trajectory_vector_.resize(HUMANOID_DOF);

  if (AssignTopicName() != 0) {
    ROS_ERROR_STREAM("Error assigning topic name");
  }

  if (InitializeTrajectory() != 0) {
    ROS_ERROR_STREAM("Error with initial trajectory");
  }

  // Get rate from Para
  nh_->param("traj_gen_rate", node_rate_, 100.0);

  callback_queue_thread_.reset(new std::thread( &Node::ServiceQueueThread, this));

  // Advertise Service
  AdvertiseServices();

}

void Node::Run()  {
  ros::Rate r(node_rate_);

  ROS_INFO_STREAM("Running at " << node_rate_<< " Hz ...");

  while (ros::ok()) {

    ROS_INFO_STREAM((msg_vector_.at(30)).data);

    if (GetLastPoint() != 0)
      ROS_ERROR_STREAM("Error getting last point in trajectory");


    if (PublishWBTrajectory() != 0)
      ROS_ERROR_STREAM("Error Publishing trajectory");


    ros::spinOnce();
    r.sleep();

  }
}

int Node::JointLinearInterpolation(size_t joint_id, double final_point, unsigned int n_steps) {

  double tolerance = 0.005; //radians
  double step;

  double init_point = trajectory_vector_.at(joint_id).back();

  final_point = deg2rad(final_point);

  step = (final_point - init_point)/n_steps;

  while (std::abs(final_point - trajectory_vector_.at(joint_id).back() ) >= tolerance) {
    trajectory_vector_.at(joint_id).push(trajectory_vector_.at(joint_id).back() + step);
  }

  return 0;
}

int Node::WBJointLinearInterpolation(std::vector<size_t>& joint_id, std::vector<double>& final_point, std::vector<unsigned int>& n_steps) {

  double tolerance = 0.005; //radians
  unsigned int default_step = node_rate_;
  std::vector<double> init_point;
  std::vector<double> step;
  std::vector<bool> joint_interp_complete;
  bool interp_complete = false;


  for (unsigned long ii=0; ii < joint_id.size(); ii++) {
    init_point.push_back(trajectory_vector_.at(joint_id.at(ii)).back());
    if (n_steps.at(ii) != 0)
      step.push_back((deg2rad(final_point.at(ii)) - init_point.at(ii))/n_steps.at(ii));
    else
      step.push_back(default_step);

    joint_interp_complete.push_back(false);
  }

  while (!interp_complete) {
    for (unsigned long ii=0; ii < joint_id.size(); ii++) {
      if (std::abs(deg2rad(final_point.at(ii)) - trajectory_vector_.at(joint_id.at(ii)).back() ) >= tolerance) {
        trajectory_vector_.at(joint_id.at(ii)).push(trajectory_vector_.at(joint_id.at(ii)).back() + step.at(ii));
        //ROS_INFO_STREAM(trajectory_vector_.at(joint_id.at(ii)).back());
      }
      else {
        trajectory_vector_.at(joint_id.at(ii)).push(trajectory_vector_.at(joint_id.at(ii)).back());
        joint_interp_complete.at(ii) = true;
      }

      // Evaluate if all interpolations are complete
      if (ii==0)
        interp_complete = joint_interp_complete.at(ii);
      else
        interp_complete = interp_complete && joint_interp_complete.at(ii);
    }
  }

  return 0;
}
int Node::PreRecordedTrajectory(PreRecordedTrajectoryName trajectory_name) {

  std::vector<size_t> joint_id_vector;
  std::vector<double> final_position_vector;
  std::vector<unsigned int> n_steps_vector;


  if (trajectory_name == PreRecordedTrajectoryName::OnlyRShLat_0_90_0) {
    size_t joint_id;
    double final_position;
    unsigned int number_steps;

    joint_id = 31;
    CleanJointTrajectory(joint_id);
    final_position = 0;
    number_steps = 3.0*node_rate_;
    JointLinearInterpolation(joint_id , final_position, number_steps);

    final_position = -90;
    number_steps = 5.0*node_rate_;
    JointLinearInterpolation(joint_id , final_position, number_steps);


    number_steps = 2.0*node_rate_;
    RepeatJointPosition(joint_id, number_steps);

    final_position = 0;
    number_steps = 5.0*node_rate_;
    JointLinearInterpolation(joint_id , final_position, number_steps);

  }

  return 0;
}

int Node::SetPose(PoseName pose_name, unsigned int n_steps, bool clean_prev_traj) {

  std::vector<size_t> joint_id_vector;
  std::vector<double> final_position_vector;
  std::vector<unsigned int> n_steps_vector;


  if (pose_name == PoseName::NPose) {
    for (int ii=6; ii <= 36; ii++) {
      joint_id_vector.push_back(ii);
      final_position_vector.push_back(0.0);
      n_steps_vector.push_back(n_steps);
    }
    //for (int ii=0; ii < joint_id_vector.size(); ii++) {
    //  ROS_INFO_STREAM("Final Position "<< ii <<" :" << final_position_vector.at(ii));
    //}

    // Stop Previous Trajectory
    if (clean_prev_traj)
      CleanAllTrajectories();

    WBJointLinearInterpolation(joint_id_vector, final_position_vector, n_steps_vector);
  }
  else if (pose_name == PoseName::TPose) {
    for (int ii=6; ii <= 36; ii++) {
      joint_id_vector.push_back(ii);
      final_position_vector.push_back(0);
      n_steps_vector.push_back(n_steps);
    }
    // Starting from ID 6
    // Left Arm
    final_position_vector.at(21 - 6) = 0;
    //final_position_vector.at(22 - 6) = PI/2;
    final_position_vector.at(22 - 6) = 90;
    final_position_vector.at(23 - 6) = 0;
    final_position_vector.at(24 - 6) = 0;
    final_position_vector.at(25 - 6) = 0;
    final_position_vector.at(26 - 6) = 0;
    final_position_vector.at(27 - 6) = 0;
    // Right Arm
    final_position_vector.at(30 - 6) = 0;
    //final_position_vector.at(31 - 6) = -PI/2;
    final_position_vector.at(31 - 6) = -90;
    final_position_vector.at(32 - 6) = 0;
    final_position_vector.at(33 - 6) = 0;
    final_position_vector.at(34 - 6) = 0;
    final_position_vector.at(35 - 6) = 0;
    final_position_vector.at(36 - 6) = 0;

    //for (int ii=0; ii < joint_id_vector.size(); ii++) {
    //  ROS_INFO_STREAM("Final Position "<< ii <<" :" << final_position_vector.at(ii));
    //}

    // Stop Previous Trajectory
    if (clean_prev_traj)
      CleanAllTrajectories();

    WBJointLinearInterpolation(joint_id_vector, final_position_vector, n_steps_vector);
  }
  else if (pose_name == PoseName::APose) {
    for (int ii=6; ii <= 36; ii++) {
      joint_id_vector.push_back(ii);
      final_position_vector.push_back(0);
      n_steps_vector.push_back(n_steps);
    }
    // Starting from ID 6
    // Left Arm
    //final_position_vector.at(21 - 6) = -PI/2;
    final_position_vector.at(21 - 6) = -79;
    final_position_vector.at(22 - 6) = 60;
    final_position_vector.at(23 - 6) = 45;
    final_position_vector.at(24 - 6) = 0;
    final_position_vector.at(25 - 6) = 0;
    final_position_vector.at(26 - 6) = 0;
    final_position_vector.at(27 - 6) = 0;
    // Right Arm
    //final_position_vector.at(30 - 6) = -PI/2;
    final_position_vector.at(30 - 6) = -79;
    final_position_vector.at(31 - 6) = -60;
    final_position_vector.at(32 - 6) = -45;
    final_position_vector.at(33 - 6) = 0;
    final_position_vector.at(34 - 6) = 0;
    final_position_vector.at(35 - 6) = 0;
    final_position_vector.at(36 - 6) = 0;

    //for (int ii=0; ii < joint_id_vector.size(); ii++) {
    //  ROS_INFO_STREAM("Final Position "<< ii <<" :" << final_position_vector.at(ii));
    //}

    // Stop Previous Trajectory
    if (clean_prev_traj)
      CleanAllTrajectories();

    WBJointLinearInterpolation(joint_id_vector, final_position_vector, n_steps_vector);
  }
  else if (pose_name == PoseName::FPose) {
    for (int ii=6; ii <= 36; ii++) {
      joint_id_vector.push_back(ii);
      final_position_vector.push_back(0);
      n_steps_vector.push_back(n_steps);
    }
    // Starting from ID 6
    // Left Arm
    //final_position_vector.at(21 - 6) = -PI/2;
    final_position_vector.at(21 - 6) = 0;
    final_position_vector.at(22 - 6) = 90;
    final_position_vector.at(23 - 6) = 90;
    final_position_vector.at(24 - 6) = -90;
    final_position_vector.at(25 - 6) = 0;
    final_position_vector.at(26 - 6) = 0;
    final_position_vector.at(27 - 6) = 0;
    // Right Arm
    //final_position_vector.at(30 - 6) = -PI/2;
    final_position_vector.at(30 - 6) = 0;
    final_position_vector.at(31 - 6) = -90;
    final_position_vector.at(32 - 6) = -90;
    final_position_vector.at(33 - 6) = -90;
    final_position_vector.at(34 - 6) = 0;
    final_position_vector.at(35 - 6) = 0;
    final_position_vector.at(36 - 6) = 0;

    //for (int ii=0; ii < joint_id_vector.size(); ii++) {
    //  ROS_INFO_STREAM("Final Position "<< ii <<" :" << final_position_vector.at(ii));
    //}

    // Stop Previous Trajectory
    if (clean_prev_traj)
      CleanAllTrajectories();

    WBJointLinearInterpolation(joint_id_vector, final_position_vector, n_steps_vector);
  }
  else if (pose_name == PoseName::BPose) {
    for (int ii=6; ii <= 36; ii++) {
      joint_id_vector.push_back(ii);
      final_position_vector.push_back(0);
      n_steps_vector.push_back(n_steps);
    }
    // Starting from ID 6
    // Left Arm
    //final_position_vector.at(21 - 6) = -PI/2;
    final_position_vector.at(21 - 6) = 0;
    final_position_vector.at(22 - 6) = 90;
    final_position_vector.at(23 - 6) = 0;
    final_position_vector.at(24 - 6) = -90;
    final_position_vector.at(25 - 6) = 0;
    final_position_vector.at(26 - 6) = 0;
    final_position_vector.at(27 - 6) = 0;
    // Right Arm
    //final_position_vector.at(30 - 6) = -PI/2;
    final_position_vector.at(30 - 6) = 0;
    final_position_vector.at(31 - 6) = -90;
    final_position_vector.at(32 - 6) = 0;
    final_position_vector.at(33 - 6) = -90;
    final_position_vector.at(34 - 6) = 0;
    final_position_vector.at(35 - 6) = 0;
    final_position_vector.at(36 - 6) = 0;

    //for (int ii=0; ii < joint_id_vector.size(); ii++) {
    //  ROS_INFO_STREAM("Final Position "<< ii <<" :" << final_position_vector.at(ii));
    //}

    // Stop Previous Trajectory
    if (clean_prev_traj)
      CleanAllTrajectories();

    WBJointLinearInterpolation(joint_id_vector, final_position_vector, n_steps_vector);
  }


  return 0;
}

bool Node::WBJointLinearInterpolationSrv(walkman_ros_trajgen::WBJointLinearInterpolation::Request &req,
                                       walkman_ros_trajgen::WBJointLinearInterpolation::Response &res) {

  //std::vector<unsigned int> n_steps;
  //std::vector<size_t> joint_id;
  //std::vector<double> final_position;


  //for (unsigned long ii=0; ii < req.wb_joints_parameters.size(); ii++) {
  //  joint_id.push_back(req.wb_joints_parameters.at(ii).joint_id);
  //  final_position.push_back(req.wb_joints_parameters.at(ii).final_position);
  //  n_steps.push_back(req.wb_joints_parameters.at(ii).time*node_rate_);
  //}

  //ROS_INFO_STREAM("Iterpolating " << n_steps.size() << " joints.");

  //if(WBJointLinearInterpolation(joint_id, final_position, n_steps))
  //  res.success = 0;
  //else
  //  res.success = 1;
  //
  //res.status_message = "OK: Time WB Joint Linear Interpolation";

    //TODO: Evaluate the contents of number_steps and time
    //TODO: Fir now only considering time
    std::vector<unsigned int> n_steps;


    for (unsigned long ii=0; ii < req.time.size(); ii++) {
      n_steps.push_back(req.time.at(ii)*node_rate_);
    }

    ROS_INFO_STREAM("Iterpolating " << n_steps.size() << " joints.");


    // Stop Previous Trajectory
    CleanAllTrajectories();

    if(WBJointLinearInterpolation(req.joint_id , req.final_position, n_steps))
      res.success = 0;
    else
      res.success = 1;

    res.status_message = "OK: Time WB Joint Linear Interpolation";

  return true;

}



bool Node::JointLinearInterpolationSrv(walkman_ros_trajgen::JointLinearInterpolation::Request &req,
                                       walkman_ros_trajgen::JointLinearInterpolation::Response &res) {

  if (req.number_steps != 0) {

    // Stop Previous Trajectory
    CleanJointTrajectory(req.joint_id);

    if(JointLinearInterpolation(req.joint_id , req.final_position, req.number_steps))
      res.success = 0;
    else
      res.success = 1;

    res.status_message = "OK: Number of steps Joint Linear Interpolation";
  }
  else if (req.time != 0) {
    unsigned int n_steps = req.time*node_rate_;

    // Stop Previous Trajectory
    CleanJointTrajectory(req.joint_id);

    if(JointLinearInterpolation(req.joint_id , req.final_position, n_steps))
      res.success = 0;
    else
      res.success = 1;

    res.status_message = "OK: Time Joint Linear Interpolation";
  }
  else {
    res.success = 0;
    res.status_message = "ERROR: No time nor n_steps specified";
    return false;
  }


  ROS_INFO_STREAM("New Linear Trajectory for joint_id:" << req.joint_id );

  return true;

}

bool Node::JointLinearTrajectorySrv(walkman_ros_trajgen::JointLinearTrajectory::Request &req,
                                       walkman_ros_trajgen::JointLinearTrajectory::Response &res) {

    std::vector<unsigned int> n_steps;

    for (unsigned ii=0; ii < req.time_array.size(); ii++) {
      n_steps.push_back(req.time_array.at(ii)*node_rate_);
    }

    // Stop Previous Trajectory
    CleanJointTrajectory(req.joint_id);

    if(JointLinearTrajectory(req.joint_id , req.point_array, n_steps))
      res.success = 0;
    else
      res.success = 1;

    res.status_message = "OK: Time Joint Linear Trajectory";

  ROS_INFO_STREAM("New Linear Trajectory for joint_id:" << req.joint_id );

  return true;

}

bool Node::SetPoseSrv(walkman_ros_trajgen::SetPose::Request &req,
                     walkman_ros_trajgen::SetPose::Response &res) {

  PoseName  pose_name;

  if (!req.pose_name.compare("NPose")) {
    pose_name = PoseName::NPose;
    ROS_INFO_STREAM("Setting NPose");
  }
  else if (!req.pose_name.compare("TPose")) {
    pose_name = PoseName::TPose;
    ROS_INFO_STREAM("Setting TPose");
  }
  else if (!req.pose_name.compare("APose")) {
    pose_name = PoseName::APose;
    ROS_INFO_STREAM("Setting APose");
  }
  else if (!req.pose_name.compare("FPose")) {
    pose_name = PoseName::FPose;
    ROS_INFO_STREAM("Setting FPose");
  }
  else if (!req.pose_name.compare("BPose")) {
    pose_name = PoseName::BPose;
    ROS_INFO_STREAM("Setting BPose");
  }
  else {
    res.success = 0;
    res.status_message = "ERROR: No such pose";
    return false;
  }

  if (req.time != 0) {
    unsigned int n_steps = req.time*node_rate_;

    if(SetPose(pose_name, n_steps, true))
      res.success = 0;
    else
      res.success = 1;

    res.status_message = "OK: Time Joint Pose";
  }
  else {
    res.success = 0;
    res.status_message = "ERROR: No time specified";
    return false;
  }

  ROS_INFO_STREAM("New Pose Set OK");

  return true;

}
bool Node::PoseInterpolationSrv(walkman_ros_trajgen::PoseInterpolation::Request &req,
                      walkman_ros_trajgen::PoseInterpolation::Response &res) {
  PoseName pose_name;
  unsigned int n_steps;
  unsigned int n_steps_between_poses;


  CleanAllTrajectories(); //TODO: This should not be here

  for (unsigned int ii=0; ii < req.pose_array.size(); ii++) {
    if (req.pose_array.at(ii) == 'N') {
      pose_name = PoseName::NPose;
      ROS_INFO_STREAM("Setting NPose");
    }
    else if (req.pose_array.at(ii) == 'T') {
      pose_name = PoseName::TPose;
      ROS_INFO_STREAM("Setting TPose");
    }
    else if (req.pose_array.at(ii) == 'A') {
      pose_name = PoseName::APose;
      ROS_INFO_STREAM("Setting APose");
    }
    else if (req.pose_array.at(ii) == 'F') {
      pose_name = PoseName::FPose;
      ROS_INFO_STREAM("Setting FPose");
    }
    else if (req.pose_array.at(ii) == 'B') {
      pose_name = PoseName::BPose;
      ROS_INFO_STREAM("Setting BPose");
    }
    else {
      res.success = 0;
      res.status_message = "ERROR: No such pose";
      return false;
    }

    if (req.time_array.at(ii) != 0) {
      n_steps = req.time_array.at(ii) * node_rate_;

      if (SetPose(pose_name, n_steps, false))
        res.success = 0;
      else
        res.success = 1;

      res.status_message = "OK: Time Joint Pose";
    }
    else {
      res.success = 0;
      res.status_message = "ERROR: No time specified";
      return false;
    }
    if (req.time_between_poses > 0) {
      n_steps_between_poses = req.time_between_poses * node_rate_;
      RepeatWBJointPosition(n_steps_between_poses);
    }

  }

  ROS_INFO_STREAM("New Pose Set OK");

  return true;
}

bool Node::PreRecordedTrajSrv(walkman_ros_trajgen::PreRecordedTrajectory::Request &req,
                              walkman_ros_trajgen::PreRecordedTrajectory::Response &res) {

  res.success = 0;

  res.status_message = "NO YET IMPLEMENTED";

  return true;
}


int Node::CleanAllTrajectories() {
  float current_point;
  for (int ii=0; ii <= 36; ii++) {
    current_point = trajectory_vector_.at(ii).front();
    std::queue<float>().swap(trajectory_vector_.at(ii));
    trajectory_vector_.at(ii).push(current_point);
  }
  return 0;
}
int Node::CleanJointTrajectory(size_t joint_id) {
  float current_point;

  current_point = trajectory_vector_.at(joint_id).front();
  std::queue<float>().swap(trajectory_vector_.at(joint_id));
  trajectory_vector_.at(joint_id).push(current_point);

  return 0;
}

int Node::InitializeTrajectory() {
  for (int ii=0; ii <= 36; ii++) {
    trajectory_vector_.at(ii).push(0);
  }

  return 0;
}

int Node::GetLastPoint() {
  for (int ii=6; ii <= 36; ii++) {
    msg_vector_.at(ii).data = (trajectory_vector_.at(ii)).front();

    if (trajectory_vector_.at(ii).size() > 1)
      trajectory_vector_.at(ii).pop();
  }

  return 0;
}

void Node::ServiceQueueThread() {
  static const double timeout = 0.001;
  while (nh_->ok()) {
    service_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void Node::AdvertiseServices() {

  // Advertise JointLinearInterpolation service
  std::string joint_linear_interp_name = "joint_linear_interpolation";
  ros::AdvertiseServiceOptions joint_linear_interp_aso =
      ros::AdvertiseServiceOptions::create<walkman_ros_trajgen::JointLinearInterpolation>(
          joint_linear_interp_name,
          boost::bind(&Node::JointLinearInterpolationSrv, this, _1, _2),
          ros::VoidPtr(), &service_queue_);
  joint_lin_interp_service_ = nh_->advertiseService(joint_linear_interp_aso);

  // Advertise WBJointLinearInterpolation service
  std::string wb_joint_linear_interp_name = "wb_joint_linear_interpolation";
  ros::AdvertiseServiceOptions wb_joint_linear_interp_aso =
      ros::AdvertiseServiceOptions::create<walkman_ros_trajgen::WBJointLinearInterpolation>(
          wb_joint_linear_interp_name,
          boost::bind(&Node::WBJointLinearInterpolationSrv, this, _1, _2),
          ros::VoidPtr(), &service_queue_);
  wb_joint_lin_interp_service_ = nh_->advertiseService(wb_joint_linear_interp_aso);

  // Advertise SetPose service
  std::string set_pose_name = "/set_pose";
  ros::AdvertiseServiceOptions set_pose_aso =
      ros::AdvertiseServiceOptions::create<walkman_ros_trajgen::SetPose>(
          set_pose_name,
          boost::bind(&Node::SetPoseSrv, this, _1, _2),
          ros::VoidPtr(), &service_queue_);
  set_pose_service_ = nh_->advertiseService(set_pose_aso);


  // Advertise RunPreRecorded Trajectory service
  std::string pre_recorded_trajectory_name = "run_pre_recorded_trajectory";
  ros::AdvertiseServiceOptions pre_recorded_traj_aso =
      ros::AdvertiseServiceOptions::create<walkman_ros_trajgen::PreRecordedTrajectory>(
          pre_recorded_trajectory_name,
          boost::bind(&Node::PreRecordedTrajSrv, this, _1, _2),
          ros::VoidPtr(), &service_queue_);
  pre_recorded_traj_service_ = nh_->advertiseService(pre_recorded_traj_aso);


  // Advertise JointLinearTrajectory service
  std::string joint_linear_traj_name = "joint_linear_trajectory";
  ros::AdvertiseServiceOptions joint_linear_traj_aso =
      ros::AdvertiseServiceOptions::create<walkman_ros_trajgen::JointLinearTrajectory>(
          joint_linear_traj_name,
          boost::bind(&Node::JointLinearTrajectorySrv, this, _1, _2),
          ros::VoidPtr(), &service_queue_);
  joint_lin_traj_service_ = nh_->advertiseService(joint_linear_traj_aso);

  // Advertise PoseInterpolation service
  std::string pose_interp_name = "pose_interpolation";
  ros::AdvertiseServiceOptions pose_interp_aso =
      ros::AdvertiseServiceOptions::create<walkman_ros_trajgen::PoseInterpolation>(
          pose_interp_name,
          boost::bind(&Node::PoseInterpolationSrv, this, _1, _2),
          ros::VoidPtr(), &service_queue_);
  joint_lin_traj_service_ = nh_->advertiseService(pose_interp_aso);

}

int Node::AssignTopicName() {

  // Left Leg
  publisher_vector_.at(6) = nh_->advertise<std_msgs::Float64>("/bigman/LHipLat_position_controller/command", 10);
  publisher_vector_.at(7) = nh_->advertise<std_msgs::Float64>("/bigman/LHipYaw_position_controller/command", 10);
  publisher_vector_.at(8) = nh_->advertise<std_msgs::Float64>("/bigman/LHipSag_position_controller/command", 10);
  publisher_vector_.at(9) = nh_->advertise<std_msgs::Float64>("/bigman/LKneeSag_position_controller/command", 10);
  publisher_vector_.at(10) = nh_->advertise<std_msgs::Float64>("/bigman/LAnkSag_position_controller/command", 10);
  publisher_vector_.at(11) = nh_->advertise<std_msgs::Float64>("/bigman/LAnkLat_position_controller/command", 10);


  // Right Leg
  publisher_vector_.at(12) = nh_->advertise<std_msgs::Float64>("/bigman/RHipLat_position_controller/command", 10);
  publisher_vector_.at(13) = nh_->advertise<std_msgs::Float64>("/bigman/RHipYaw_position_controller/command", 10);
  publisher_vector_.at(14) = nh_->advertise<std_msgs::Float64>("/bigman/RHipSag_position_controller/command", 10);
  publisher_vector_.at(15) = nh_->advertise<std_msgs::Float64>("/bigman/RKneeSag_position_controller/command", 10);
  publisher_vector_.at(16) = nh_->advertise<std_msgs::Float64>("/bigman/RAnkSag_position_controller/command", 10);
  publisher_vector_.at(17) = nh_->advertise<std_msgs::Float64>("/bigman/RAnkLat_position_controller/command", 10);

  // Waist
  publisher_vector_.at(18) = nh_->advertise<std_msgs::Float64>("/bigman/WaistLat_position_controller/command", 10);
  publisher_vector_.at(19) = nh_->advertise<std_msgs::Float64>("/bigman/WaistSag_position_controller/command", 10);
  publisher_vector_.at(20) = nh_->advertise<std_msgs::Float64>("/bigman/WaistYaw_position_controller/command", 10);

  // Left Arm
  publisher_vector_.at(21) = nh_->advertise<std_msgs::Float64>("/bigman/LShSag_position_controller/command", 10);
  publisher_vector_.at(22) = nh_->advertise<std_msgs::Float64>("/bigman/LShLat_position_controller/command", 10);
  publisher_vector_.at(23) = nh_->advertise<std_msgs::Float64>("/bigman/LShYaw_position_controller/command", 10);
  publisher_vector_.at(24) = nh_->advertise<std_msgs::Float64>("/bigman/LElbj_position_controller/command", 10);
  publisher_vector_.at(25) = nh_->advertise<std_msgs::Float64>("/bigman/LForearmPlate_position_controller/command", 10);
  publisher_vector_.at(26) = nh_->advertise<std_msgs::Float64>("/bigman/LWrj1_position_controller/command", 10);
  publisher_vector_.at(27) = nh_->advertise<std_msgs::Float64>("/bigman/LWrj2_position_controller/command", 10);

  // Neck
  publisher_vector_.at(28) = nh_->advertise<std_msgs::Float64>("/bigman/LWrj1_position_controller/command", 10);
  publisher_vector_.at(29) = nh_->advertise<std_msgs::Float64>("/bigman/LWrj2_position_controller/command", 10);


  // Right Arm
  publisher_vector_.at(30) = nh_->advertise<std_msgs::Float64>("/bigman/RShSag_position_controller/command", 10);
  publisher_vector_.at(31) = nh_->advertise<std_msgs::Float64>("/bigman/RShLat_position_controller/command", 10);
  publisher_vector_.at(32) = nh_->advertise<std_msgs::Float64>("/bigman/RShYaw_position_controller/command", 10);
  publisher_vector_.at(33) = nh_->advertise<std_msgs::Float64>("/bigman/RElbj_position_controller/command", 10);
  publisher_vector_.at(34) = nh_->advertise<std_msgs::Float64>("/bigman/RForearmPlate_position_controller/command", 10);
  publisher_vector_.at(35) = nh_->advertise<std_msgs::Float64>("/bigman/RWrj1_position_controller/command", 10);
  publisher_vector_.at(36) = nh_->advertise<std_msgs::Float64>("/bigman/RWrj2_position_controller/command", 10);

  return 0;
}

int Node::PublishWBTrajectory () {

  for (int ii=6; ii <= 36; ii++) {
    (publisher_vector_.at(ii)).publish(msg_vector_.at(ii));
  }

  return 0;
}

int Node::RepeatJointPosition(size_t joint_id, unsigned int n_times) {

  for (unsigned int ii=1; ii <= n_times; ii++) {
    trajectory_vector_.at(joint_id).push(trajectory_vector_.at(joint_id).back());
  }
  return 0;
}

int Node::RepeatWBJointPosition(unsigned int n_times) {

  for (unsigned int ii=1; ii <= n_times; ii++) {
    for (int jj=6; jj <= 36; jj++) {
      trajectory_vector_.at(jj).push(trajectory_vector_.at(jj).back());
    }
  }
  return 0;
}

int Node::JointLinearTrajectory(size_t joint_id, std::vector<double> &point_array, std::vector<unsigned int> &n_steps) {

  double tolerance = 0.1; //degrees
  for (unsigned int ii=0; ii < point_array.size(); ii++) {
    if (ii > 1) {
      if (std::abs(point_array.at(ii) - point_array.at(ii-1)) >= tolerance)
        JointLinearInterpolation(joint_id, point_array.at(ii), n_steps.at(ii));
      else
        RepeatJointPosition(joint_id, n_steps.at(ii));
    }
    else {
      if (std::abs(point_array.at(ii) - rad2deg(trajectory_vector_.at(joint_id).back())) >= tolerance)
        JointLinearInterpolation(joint_id, point_array.at(ii), n_steps.at(ii));
      else
        RepeatJointPosition(joint_id, n_steps.at(ii));
    }
  }

  return 0;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "joint_traj_gen");

    Node node;
    node.Run();

    return 0;
}
