/*
 * File:   PID.h
 */

#ifndef AFC_H
#define AFC_H
#define _USE_MATH_DEFINES

#include "ros/ros.h"
#include <vector>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <stdlib.h>
#include <interbotix_xs_msgs/JointGroupCommand.h>
#include <interbotix_xs_msgs/JointSingleCommand.h>

// Class PID to hanle the subscribers and the publishers for the active inference controller
class AFC
{
public:
  // Constructor and destructor
  AFC(int whichRobot);
  ~AFC();

  // Callback to handle the proprioceptive sensory data from the topic /joint_states published at 1kHz
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
  // Method to set the necessary variables once the constructor is called
  void initVariables();
  // Calculate and send the torque commands to compute actions and further minimise the free-energy
  void computeActions();
  // Support method to control the program flow. Data ready returns one when the encoders has been read
  int dataReady();
  // Set desired position
  void setGoal(std::vector<double> desiredPos);
  // get methods for sensory prediction errors
  void signum_1();

  void setStep(std::vector<double> controlInput);

private:

  // Variances associated with the active inference controller and the confidence relative to sensory input and beliefs
  double k_p, k_d, k_i, p, lam;
  // Precision matrices, diagonal matrices with the inverce of the variance
  Eigen::Matrix<double, 5, 5> Kp, Kd, Ki, P, lambda;
  // Beliefs about the states and their derivatives mu, mu', mu'', column vectors of 7 elements
  Eigen::Matrix<double, 5, 1> jointPos, jointVel;
  // Desired robot's states, column vector of 7 elements
  Eigen::Matrix<double, 5, 1> mu_d, mu_p_d, error, error_p, se, k_c_dot, k_c_dot_prev, k_c, k_c_prev, sigma;
  // Control actions,  column vector of 7 elements
  Eigen::Matrix<double, 5, 1> u;
  // Learning rates and integration step for the PID
  double h;
  // Support variable to control the flow of the script
  int dataReceived;
  // ROS related Variables, node handle
  ros::NodeHandle nh;
  // Publishers for joint torques to the topics /panda_joint*_controller/command, and the free-energy
  ros::Publisher singlePub, groupPub, tauPub1, tauPub2, tauPub3, tauPub4, tauPub5, tauPub6, tauPub7, mu_desired_pub;
  // Subscriber for proprioceptive sensors (i.e. from joint_states) and camera (i.e. aruco_single/pose)
  ros::Subscriber sensorSub;
  // Support variables to contain the torques for the joints
  std_msgs::Float64 tau1, tau2, tau3, tau4, tau5, tau6, tau7;

  std_msgs::Float64MultiArray mu_des;

};

#endif
