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
#include <mutex>
#include <cmath> // M_PI
#include <Eigen/Dense>
#include <memory> //shared_ptr
#include <functional> // bind
#include <fstream>
#include <iomanip>      // std::setprecision

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>


#include "walkman_ros_trajgen/JointLinearInterpolation.h"
#include "walkman_ros_trajgen/WBJointLinearInterpolation.h"
#include "walkman_ros_trajgen/PoseInterpolation.h"
#include "walkman_ros_trajgen/JointLinearTrajectory.h"
#include "walkman_ros_trajgen/SetPose.h"
#include "walkman_ros_trajgen/PreRecordedTrajectory.h"

#define HUMANOID_DOF 37


#define deg2rad(angleDegrees) (angleDegrees * M_PI / 180.0)
#define rad2deg(angleRadians) (angleRadians * 180.0 / M_PI)


//enum class InterpolationType { linear, splite, polynomial5 };
enum class PoseName{TPose, NPose, APose, FPose, BPose, MPose};
enum class PreRecordedTrajectoryName{OnlyRShLat_0_90_0};
enum class InterpolationMethod{linear, polynomial5};

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> ColumnVector6d;
typedef Eigen::Matrix<double, 5, 1> ColumnVector5d;
typedef Eigen::Matrix<double, 4, 1> ColumnVector4d;
typedef Eigen::Matrix<double, 1, 6> RowVector6d;
typedef Eigen::Matrix<double, 1, 5> RowVector5d;
typedef Eigen::Matrix<double, 1, 4> RowVector4d;
typedef Eigen::Matrix<double, 1, 3> RowVector3d;


class Node {

public:
    Node();

    ~Node() {

    }

    void Run();

private:
    // Trajectories
    std::vector<std::queue<float>> trajectory_vector_;
    int GetLastPoint();
    int InitializeTrajectory();

    int Polynomial5Interpolation(double x0, double xf, double dx0, double dxf, double ddx0, double ddxf, unsigned int n, std::vector<double>&xtraj, std::vector<double>&dxtraj);
    int JointLinearInterpolation(size_t joint_id, double final_point, unsigned int n_steps);
    int JointLinearTrajectory(size_t joint_id, std::vector<double>& point_array, std::vector<unsigned int>& n_steps);
    int WBJointLinearInterpolation(std::vector<size_t>& joint_id, std::vector<double>& final_point, std::vector<unsigned int>& n_steps);
    int WBJointPolynomial5Interpolation(std::vector<size_t>& joint_id, std::vector<double>& final_point, std::vector<double>& final_velocity, std::vector<unsigned int>& n_steps);
    int SetPose(PoseName pose_name, unsigned int n_steps, bool clean_prev_traj, InterpolationMethod interpolation_method);
    int PreRecordedTrajectory(PreRecordedTrajectoryName trajectory_name);
    int CleanAllTrajectories();
    int CleanJointTrajectory(size_t joint_id);
    int RepeatJointPosition(size_t joint_id, unsigned int n_times);
    int RepeatWBJointPosition(unsigned int n_times);




    // ROS
    std::shared_ptr<ros::NodeHandle> nh_;
    //ros::NodeHandle nh_;
    double node_rate_;
    std::vector<std_msgs::Float64> msg_vector_;
    std::vector<ros::Publisher> publisher_vector_;
    int PublishWBTrajectory();
    int AssignTopicName();

    // ROS Services
    void ServiceQueueThread();
    void AdvertiseServices();

    ros::CallbackQueue service_queue_;
    std::shared_ptr<std::thread> callback_queue_thread_;
    ros::ServiceServer joint_lin_interp_service_;
    ros::ServiceServer wb_joint_lin_interp_service_;
    ros::ServiceServer set_pose_service_;
    ros::ServiceServer pre_recorded_traj_service_;
    ros::ServiceServer joint_lin_traj_service_;
    ros::ServiceServer pose_interp_service_;

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


  // Get rate from Parameter
  nh_->param("traj_gen_rate", node_rate_, 40.0);

  // Node Services Thread
  callback_queue_thread_.reset(new std::thread( &Node::ServiceQueueThread, this));
  AdvertiseServices();

}

void Node::Run()  {
  ros::Rate r(node_rate_);

  ROS_INFO_STREAM("Running at " << node_rate_<< " Hz ...");

  //unsigned int n = 0.5*node_rate_;
  //double x0 = 1;
  //double xf = 3;
  //double dx0 = 0;
  //double dxf = 0;
  //double ddx0 = 0;
  //double ddxf = 0;
  //std::vector<double> xtraj;
  //std::vector<double> dxtraj;
  //Polynomial5Interpolation( x0, xf, dx0, dxf, ddx0, ddxf, n, xtraj, dxtraj);
  //for (unsigned int ii = 0; ii < xtraj.size(); ii++){
  //  std::cout << xtraj.at(ii) << ", ";
  //}
  //std::cout << std::endl;
  //for (unsigned int ii = 0; ii < dxtraj.size(); ii++){
  //  std::cout << dxtraj.at(ii) << ", ";
  //}
  //std::cout << std::endl;

  while (ros::ok()) {

    //ROS_INFO_STREAM((msg_vector_.at(30)).data);

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

int Node::WBJointPolynomial5Interpolation(std::vector<size_t>& joint_id, std::vector<double>& final_point, std::vector<double>& final_velocity, std::vector<unsigned int>& n_steps) {

  // Calculate The Coefficients
  Matrix6d A;
  ColumnVector6d b;
  std::vector<ColumnVector6d> coeffs_vect;
  std::vector<ColumnVector5d> dcoeffs_vect;
  ColumnVector6d coeffs;
  ColumnVector5d dcoeffs;
  unsigned int n;
  double x0, xf, dx0, dxf, ddx0, ddxf;

  std::vector<bool> joint_interp_complete;
  std::vector<unsigned int> current_n_vect;
  bool interp_complete = false;
  double request_joint_angle;

  ddx0 = 0;
  ddxf = 0;

  for (unsigned int ii = 0; ii < final_point.size(); ii++) {
    n = n_steps.at(ii);

    x0 = trajectory_vector_.at(joint_id.at(ii)).back();

    xf = deg2rad(final_point.at(ii));

    dx0 = 0; //TODO : Is it ok to assume this?

    dxf = deg2rad(final_velocity.at(ii));

    A << 0,              0,              0,        0,       0,    1,
        pow(n,5),       pow(n,4),     pow(n,3), pow(n,2),   n,    1,
        0,              0,             0,        0,       1,    0,
        5*pow(n,4),   4*pow(n,3),   3*pow(n,2),   2*n,      1,    0,
        0,              0,           0,          2,       0,    0,
        20*pow(n,3),  12*pow(n,2),    6*n,         2,       0,    0;
    b << x0, xf, dx0, dxf, 0, 0;
    coeffs = A.colPivHouseholderQr().solve(b);
    coeffs_vect.push_back(coeffs);

    dcoeffs << 5*coeffs_vect.at(ii)(0,0), 4*coeffs_vect.at(ii)(1,0), 3*coeffs_vect.at(ii)(2,0), 2*coeffs_vect.at(ii)(3,0), coeffs_vect.at(ii)(4,0);
    dcoeffs_vect.push_back(dcoeffs);

    //ROS_WARN_STREAM(x0 << " "<< xf << " "<< dx0  << " "<<dxf << " " << ddx0 << " " << ddxf);

    joint_interp_complete.push_back(false);
    current_n_vect.push_back(0);
  }


  while (!interp_complete) {
    for (unsigned long ii=0; ii < joint_id.size(); ii++) {

      if (current_n_vect.at(ii) <= n_steps.at(ii)) {
        trajectory_vector_.at(joint_id.at(ii)).push(coeffs_vect.at(ii)(0,0)*pow(current_n_vect.at(ii),5) + coeffs_vect.at(ii)(1,0)*pow(current_n_vect.at(ii),4) + coeffs_vect.at(ii)(2,0)*pow(current_n_vect.at(ii),3) + coeffs_vect.at(ii)(3,0)*pow(current_n_vect.at(ii),2) + coeffs_vect.at(ii)(4,0)*current_n_vect.at(ii) + coeffs_vect.at(ii)(5,0));

        //if (joint_id.at(ii) == 31) {
        //  ROS_INFO_STREAM(trajectory_vector_.at(ii).back());
        //    double velocity;
        //    velocity = coeffs_vect.at(ii)(0,0)*pow(current_n_vect.at(ii),4) + coeffs_vect.at(ii)(1,0)*pow(current_n_vect.at(ii),3) + coeffs_vect.at(ii)(2,0)*pow(current_n_vect.at(ii),2) + coeffs_vect.at(ii)(3,0)*current_n_vect.at(ii) + coeffs_vect.at(ii)(4,0);
        //    ROS_INFO_STREAM(velocity);
        //}
        current_n_vect.at(ii)+=1;
      }
      else {
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

int Node::SetPose(PoseName pose_name, unsigned int n_steps, bool clean_prev_traj, InterpolationMethod interpolation_method) {

  std::vector<size_t> joint_id_vector;
  std::vector<double> final_position_vector;
  std::vector<double> final_velocity_vector;
  std::vector<unsigned int> n_steps_vector;


  if (pose_name == PoseName::NPose) {
    for (int ii=6; ii <= 36; ii++) {
      joint_id_vector.push_back(ii);
      final_position_vector.push_back(0.0);
      final_velocity_vector.push_back(0.0);
      n_steps_vector.push_back(n_steps);
    }
    //for (int ii=0; ii < joint_id_vector.size(); ii++) {
    //  ROS_INFO_STREAM("Final Position "<< ii <<" :" << final_position_vector.at(ii));
    //}

    // Stop Previous Trajectory
    if (clean_prev_traj)
      CleanAllTrajectories();

    if (interpolation_method == InterpolationMethod::linear)
      WBJointLinearInterpolation(joint_id_vector, final_position_vector, n_steps_vector);
    else if (interpolation_method == InterpolationMethod::polynomial5)
      WBJointPolynomial5Interpolation(joint_id_vector, final_position_vector, final_velocity_vector, n_steps_vector);
  }
  else if (pose_name == PoseName::TPose) {
    for (int ii=6; ii <= 36; ii++) {
      joint_id_vector.push_back(ii);
      final_position_vector.push_back(0);
      final_velocity_vector.push_back(0.0);
      n_steps_vector.push_back(n_steps);
    }
    // Starting from ID 6
    // Left Arm
    final_position_vector.at(21 - 6) = 0;
    final_position_vector.at(22 - 6) = 90;
    final_position_vector.at(23 - 6) = 0;
    final_position_vector.at(24 - 6) = 0;
    final_position_vector.at(25 - 6) = 0;
    final_position_vector.at(26 - 6) = 0;
    final_position_vector.at(27 - 6) = 0;
    // Right Arm
    final_position_vector.at(30 - 6) = 0;
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

    if (interpolation_method == InterpolationMethod::linear)
      WBJointLinearInterpolation(joint_id_vector, final_position_vector, n_steps_vector);
    else if (interpolation_method == InterpolationMethod::polynomial5)
      WBJointPolynomial5Interpolation(joint_id_vector, final_position_vector, final_velocity_vector, n_steps_vector);
  }
  else if (pose_name == PoseName::APose) {
    for (int ii=6; ii <= 36; ii++) {
      joint_id_vector.push_back(ii);
      final_position_vector.push_back(0);
      final_velocity_vector.push_back(0.0);
      n_steps_vector.push_back(n_steps);
    }
    // Starting from ID 6
    // Left Arm
    final_position_vector.at(21 - 6) = -79;
    final_position_vector.at(22 - 6) = 60;
    final_position_vector.at(23 - 6) = 45;
    final_position_vector.at(24 - 6) = 0;
    final_position_vector.at(25 - 6) = 0;
    final_position_vector.at(26 - 6) = 0;
    final_position_vector.at(27 - 6) = 0;
    // Right Arm
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

    if (interpolation_method == InterpolationMethod::linear)
      WBJointLinearInterpolation(joint_id_vector, final_position_vector, n_steps_vector);
    else if (interpolation_method == InterpolationMethod::polynomial5)
      WBJointPolynomial5Interpolation(joint_id_vector, final_position_vector, final_velocity_vector, n_steps_vector);
  }
  else if (pose_name == PoseName::FPose) {
    for (int ii=6; ii <= 36; ii++) {
      joint_id_vector.push_back(ii);
      final_position_vector.push_back(0);
      final_velocity_vector.push_back(0.0);
      n_steps_vector.push_back(n_steps);
    }
    // Starting from ID 6
    // Left Arm
    final_position_vector.at(21 - 6) = 0;
    final_position_vector.at(22 - 6) = 90;
    final_position_vector.at(23 - 6) = 90;
    final_position_vector.at(24 - 6) = -90;
    final_position_vector.at(25 - 6) = 0;
    final_position_vector.at(26 - 6) = 0;
    final_position_vector.at(27 - 6) = 0;
    // Right Arm
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

    if (interpolation_method == InterpolationMethod::linear)
      WBJointLinearInterpolation(joint_id_vector, final_position_vector, n_steps_vector);
    else if (interpolation_method == InterpolationMethod::polynomial5)
      WBJointPolynomial5Interpolation(joint_id_vector, final_position_vector, final_velocity_vector, n_steps_vector);
  }
  else if (pose_name == PoseName::BPose) {
    for (int ii=6; ii <= 36; ii++) {
      joint_id_vector.push_back(ii);
      final_position_vector.push_back(0);
      final_velocity_vector.push_back(0.0);
      n_steps_vector.push_back(n_steps);
    }
    // Starting from ID 6
    // Left Arm
    final_position_vector.at(21 - 6) = 0;
    final_position_vector.at(22 - 6) = 90;
    final_position_vector.at(23 - 6) = 0;
    final_position_vector.at(24 - 6) = -90;
    final_position_vector.at(25 - 6) = 0;
    final_position_vector.at(26 - 6) = 0;
    final_position_vector.at(27 - 6) = 0;
    // Right Arm
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

    if (interpolation_method == InterpolationMethod::linear)
      WBJointLinearInterpolation(joint_id_vector, final_position_vector, n_steps_vector);
    else if (interpolation_method == InterpolationMethod::polynomial5)
      WBJointPolynomial5Interpolation(joint_id_vector, final_position_vector, final_velocity_vector, n_steps_vector);
  }
  else if (pose_name == PoseName::MPose) {
    for (int ii=6; ii <= 36; ii++) {
      joint_id_vector.push_back(ii);
      final_position_vector.push_back(0);
      final_velocity_vector.push_back(0.0);
      n_steps_vector.push_back(n_steps);
    }
    // Starting from ID 6
    // Left Arm
    final_position_vector.at(21 - 6) = 0;
    final_position_vector.at(22 - 6) = 90;
    final_position_vector.at(23 - 6) = -90;
    final_position_vector.at(24 - 6) = -90;
    final_position_vector.at(25 - 6) = 0;
    final_position_vector.at(26 - 6) = 0;
    final_position_vector.at(27 - 6) = 0;
    // Right Arm
    final_position_vector.at(30 - 6) = 0;
    final_position_vector.at(31 - 6) = -90;
    final_position_vector.at(32 - 6) = 90;
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

    if (interpolation_method == InterpolationMethod::linear)
      WBJointLinearInterpolation(joint_id_vector, final_position_vector, n_steps_vector);
    else if (interpolation_method == InterpolationMethod::polynomial5)
      WBJointPolynomial5Interpolation(joint_id_vector, final_position_vector, final_velocity_vector, n_steps_vector);
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

    if (!req.interpolation_method.compare("linear") || !req.interpolation_method.compare("LINEAR") || !req.interpolation_method.compare("Linear")) {
      if(WBJointLinearInterpolation(req.joint_id , req.final_position, n_steps))
        res.success = 0;
      else
        res.success = 1;
    }
    else {
      if(WBJointPolynomial5Interpolation(req.joint_id , req.final_position, req.final_velocity, n_steps))
        res.success = 0;
      else
        res.success = 1;
    }

    res.status_message = "OK: Time WB Joint Linear Interpolation";

  return true;

}



bool Node::JointLinearInterpolationSrv(walkman_ros_trajgen::JointLinearInterpolation::Request &req,
                                       walkman_ros_trajgen::JointLinearInterpolation::Response &res) {

  if (req.number_steps != 0) {

    // Stop Previous Trajectory
    CleanJointTrajectory(req.joint_id);

    if (!req.interpolation_method.compare("linear") || !req.interpolation_method.compare("LINEAR") || !req.interpolation_method.compare("Linear")) {
      if(JointLinearInterpolation(req.joint_id , req.final_position, req.number_steps))
        res.success = 0;
      else
        res.success = 1;

      res.status_message = "OK: Number of steps Joint Linear Interpolation";
    }
    else {
      if(JointLinearInterpolation(req.joint_id , req.final_position, req.number_steps))
        res.success = 0;
      else
        res.success = 1;

      res.status_message = "WARNING: No yet implemented. Using Joint Linear Interpolation";
    }
  }
  else if (req.time != 0) {
    unsigned int n_steps = req.time*node_rate_;

    // Stop Previous Trajectory
    CleanJointTrajectory(req.joint_id);

    if (!req.interpolation_method.compare("linear") || !req.interpolation_method.compare("LINEAR") || !req.interpolation_method.compare("Linear")) {
      if(JointLinearInterpolation(req.joint_id , req.final_position, n_steps))
        res.success = 0;
      else
        res.success = 1;

      res.status_message = "OK: Time Joint Linear Interpolation";
    }
    else {
      ROS_WARN_STREAM("HOLAAAA");
      if(JointLinearInterpolation(req.joint_id , req.final_position, n_steps))
        res.success = 0;
      else
        res.success = 1;

      res.status_message = "WARNING: No yet implemented. Using Joint Linear Interpolation";
    }

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
  InterpolationMethod interpolation_method;

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
  else if (!req.pose_name.compare("MPose")) {
    pose_name = PoseName::MPose;
    ROS_INFO_STREAM("Setting MPose");
  }
  else {
    res.success = 0;
    res.status_message = "ERROR: No such pose";
    return false;
  }

  if (!req.interpolation_method.compare("linear") || !req.interpolation_method.compare("LINEAR") || !req.interpolation_method.compare("Linear")) {
    interpolation_method = InterpolationMethod::linear;
  }
  else {
    interpolation_method = InterpolationMethod::polynomial5;
  }


  if (req.time != 0) {
    unsigned int n_steps = req.time*node_rate_;

    if(SetPose(pose_name, n_steps, true, interpolation_method))
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

  // By default Polynomial5 Trajectory with 0 velocity and acceleration

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
    else if (req.pose_array.at(ii) == 'M') {
      pose_name = PoseName::MPose;
      ROS_INFO_STREAM("Setting MPose");
    }
    else {
      res.success = 0;
      res.status_message = "ERROR: No such pose";
      return false;
    }

    if (req.time_array.at(ii) != 0) {
      n_steps = req.time_array.at(ii) * node_rate_;

      if (!req.interpolation_method.compare("linear") || !req.interpolation_method.compare("LINEAR") || !req.interpolation_method.compare("Linear")) {
        if (SetPose(pose_name, n_steps, false, InterpolationMethod::linear))
          res.success = 0;
        else
          res.success = 1;
      }
      else {
        if (SetPose(pose_name, n_steps, false, InterpolationMethod::polynomial5))
          res.success = 0;
        else
          res.success = 1;
      }

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
  std::string set_pose_name = "set_pose";
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

int Node::Polynomial5Interpolation(double x0, double xf, double dx0, double dxf, double ddx0, double ddxf,
                                   unsigned int n, std::vector<double> &xtraj, std::vector<double> &dxtraj) {


  Matrix6d A;
  ColumnVector6d b;
  RowVector6d coeffs;
  RowVector5d dcoeffs;
  ColumnVector6d coeffs2;
  ColumnVector5d dcoeffs2;
  //RowVector4d ddcoeffs;
  n = n-1;
  A << 0,              0,              0,        0,       0,    1,
      pow(n,5),       pow(n,4),     pow(n,3), pow(n,2),   n,    1,
        0,              0,             0,        0,       1,    0,
      5*pow(n,4),   4*pow(n,3),   3*pow(n,2),   2*n,      1,    0,
        0,              0,           0,          2,       0,    0,
      20*pow(n,3),  12*pow(n,2),    6*n,         2,       0,    0;

  b << x0, xf, dx0, dxf, ddx0, ddxf;

  //coeffs = A.inverse() * b;
  coeffs2 = A.colPivHouseholderQr().solve(b);

  //double x;
  //x = coeffs(0,5);
  //ROS_INFO_STREAM("X is... " << x);

  // Coefficients of derivatives
  //dcoeffs << 5*coeffs(0,0), 4*coeffs(0,1), 3*coeffs(0,2), 2*coeffs(0,3), coeffs(0,4);
  dcoeffs2 << 5*coeffs2(0,0), 4*coeffs2(1,0), 3*coeffs2(2,0), 2*coeffs2(3,0), coeffs2(4,0);
  //ddcoeffs << 4*dcoeffs(1,1),3*dcoeffs(1,2),2*dcoeffs(1,3),dcoeffs(1,4);

  for (unsigned int ii = 0 ; ii <= n ; ii++){
    // With Inverse
    //xtraj.push_back(coeffs(0,0)*pow(ii,5) + coeffs(0,1)*pow(ii,4) + coeffs(0,2)*pow(ii,3) + coeffs(0,3)*pow(ii,2) + coeffs(0,4)*ii + coeffs(0,5));
    //dxtraj.push_back(dcoeffs(0,0)*pow(ii,4) + dcoeffs(0,1)*pow(ii,3) + dcoeffs(0,2)*pow(ii,2) + dcoeffs(0,3)*ii + dcoeffs(0,4));
    // With Solve
    xtraj.push_back(coeffs2(0,0)*pow(ii,5) + coeffs2(1,0)*pow(ii,4) + coeffs2(2,0)*pow(ii,3) + coeffs2(3,0)*pow(ii,2) + coeffs2(4,0)*ii + coeffs2(5,0));
    dxtraj.push_back(dcoeffs2(0,0)*pow(ii,4) + dcoeffs2(1,0)*pow(ii,3) + dcoeffs2(2,0)*pow(ii,2) + dcoeffs2(3,0)*ii + dcoeffs2(4,0));
  }

  return 0;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "joint_traj_gen");

    Node node;
    node.Run();

    return 0;
}
