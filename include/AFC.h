/*
 * File: AFC.h

 * Author: Alon Dawe
 * 
 * Created: 13th October, 2023
 * 
 * Description: Class to perform Adaptive Friction Control of the 5-DOF Interbotix PincherX 150 robotic manipulator using ROS.
 * 
 * Original Author: Corrado Pezzato, TU Delft, DCSC 
 * (https://github.com/cpezzato/panda_simulation/blob/master/panda_control/include/AIC.h)
 * The original author implemented an AIC controller to control a 7-DOF Franka Emika Panda robot arm.
 * The code in this file originated from this source, and was adapted to suit the control of a 5-DOF 
 * Interbotix PincherX 150 robotic manipulator with a different control algorithm. 
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

// Class AFC to hanle the subscribers and the publishers for the AFC Controller
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
  // Calculate and send the control action commands
  void computeActions();
  // Support method to control the program flow. Data ready returns one when the encoders has been read
  int dataReady();
  // Set desired position
  void setGoal(std::vector<double> desiredPos);
  // AFC signum function for friction adaptation
  void signum_1();
  // function for the impulse response test
  void setStep(std::vector<double> controlInput);

private:

  // Tuning parameters for the AFC
  double k_p, k_d, k_i, p, lam;
  // Diagonal matrices for the tuning parameters
  Eigen::Matrix<double, 5, 5> Kp, Kd, Ki, P, lambda;
  // column vectors for the joint position and velocity
  Eigen::Matrix<double, 5, 1> jointPos, jointVel;
  // Desired robot's states, column vector of 5 elements
  Eigen::Matrix<double, 5, 1> mu_d, mu_p_d, error, error_p, se, k_c_dot, k_c_dot_prev, k_c, k_c_prev, sigma, gravity_flag;
  // Control actions,  column vector of 5 elements
  Eigen::Matrix<double, 5, 1> u, u_AFC;
  // integration step for the PID
  double h;
  // Support variable to control the flow of the script
  int dataReceived;
  // ROS related Variables, node handle
  ros::NodeHandle nh;
  // Publishers to the topics 
  ros::Publisher singlePub, groupPub, tauPub1, tauPub2, tauPub3, tauPub4, tauPub5, tauPub6, tauPub7, mu_desired_pub;
  // Subscriber for proprioceptive sensors (i.e. from joint_states)
  ros::Subscriber sensorSub;
  // Support variables to contain the torques for the joints in Gazebo
  std_msgs::Float64 tau1, tau2, tau3, tau4, tau5, tau6, tau7;
  // Definition of variables in order to publish the goal state
  std_msgs::Float64MultiArray mu_des;

};

#endif
