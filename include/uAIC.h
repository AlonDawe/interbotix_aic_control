/*
 * File: uAIC.h

 * Author: Alon Dawe
 * 
 * Created: 13th October, 2023
 * 
 * Description: Class header to perform Unbiased Active Inference Control of the 5-DOF Interbotix PincherX 150 robotic manipulator using ROS.
 * 
 * Original Author: Corrado Pezzato, TU Delft, DCSC 
 * (https://github.com/cpezzato/unbiased_aic/blob/master/include/uAIC.h)
 * The original author implemented an ubiased AIC controller to control a 7-DOF Franka Emika Panda robot arm.
 * The code in this file originated from this source, and was adapted to suit the control of a 5-DOF 
 * Interbotix PincherX 150 robotic manipulator
 */

#ifndef uAIC_H
#define uAIC_H
#define _USE_MATH_DEFINES

#include "ros/ros.h"
#include <vector>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
//#include "geometry_msgs/PoseStamped.h"
//#include "unbiased_aic/reference.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <stdlib.h>
#include <interbotix_xs_msgs/JointGroupCommand.h>
#include <interbotix_xs_msgs/JointSingleCommand.h>

// Class uAIC to hanle the subscribers and the publishers for the active inference controller
class uAIC
{
public:
  // Constructor and destructor
  uAIC(int whichRobot);
  ~uAIC();

  // Callback to handle the proprioceptive sensory data from the topic /joint_states published at 1kHz
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
  // Method to set the necessary variables once the constructor is called
  void initVariables();
  // Main method which minimises the free-energy using gradient descent
  void minimiseF();
  // Calculate and send the torque commands to compute actions and further minimise the free-energy
  void computeActions();
  // Support method to control the program flow. Data ready returns one when the encoders has been read
  int dataReady();
  // Set desired position
  void setGoal(std::vector<double> desiredPos);
  void setGoalCurrentState();
  // get methods for sensory prediction errors
  std_msgs::Float64MultiArray getSPE();
  // Getting trajectory
  //void setDesiredState(const unbiased_aic::reference::ConstPtr& msg);
  void setDesiredState(const sensor_msgs::JointState::ConstPtr& msg);

private:

  // Variances associated with the active inference controller and the confidence relative to sensory input and beliefs
  double var_q, var_qdot, var_mu, var_muprime;
  // Precision matrices, diagonal matrices with the inverce of the variance
  Eigen::Matrix<double, 5, 5> SigmaP_yq0, SigmaP_yq1, SigmaP_mu, SigmaP_muprime;
  // Controller parameters, PID like control law. Diagonal matrices
  Eigen::Matrix<double, 5, 5> K_p, K_i, K_d;
  // Beliefs about the states and their derivatives mu, mu', mu'', column vectors of 7 elements. Temp variables mu_past, mu_p_past to keep track of past time step
  Eigen::Matrix<double, 5, 1> mu, mu_p, mu_pp, mu_dot, mu_dot_p, mu_dot_pp, jointPos, jointVel, mu_past, mu_p_past;
  // Desired robot's states, column vector of 7 elements
  Eigen::Matrix<double, 5, 1> mu_d, mu_p_d;
  // Control actions,  column vector of 7 elements, and integral gain
  Eigen::Matrix<double, 5, 1> u, I_gain;
  // Parameters for control law, to populate the gain matrices
  double  k_p, k_d, k_i, max_i, k_p0, k_p1, k_p2, k_p3, k_p4;
  // Learning rates and integration step for the uAIC
  double k_mu, k_a, h;
  // Sensory prediction errors
  double SPEq, SPEdq, SPEmu_p, SPEmu_pp;
  // Support variable to control the flow of the script
  int dataReceived;
  // ROS related Variables, node handle
  ros::NodeHandle nh;
  // Publishers for joint torques to the topics /panda_joint*_controller/command, and the free-energy
  ros::Publisher singlePub, groupPub, tauPub1, tauPub2, tauPub3, tauPub4, tauPub5, torque_pub, beliefs_mu_p_pub, beliefs_mu_pub;
  // Subscriber for proprioceptive sensors (i.e. from joint_states) and camera (i.e. aruco_single/pose)
  ros::Subscriber sensorSub;
  // Support variables to contain the torques for the joints
  std_msgs::Float64 tau1, tau2, tau3, tau4, tau5, tau6, tau7, F;
  // Values for direct kinematics computation using DH parameters
  Eigen::Matrix<double, 7, 1> DH_a, DH_d, DH_alpha;
  Eigen::Matrix<double, 4, 4> DH_T, DH_A, T;
  Eigen::Matrix<double, 3, 1> eePosition;
  // Definition of variables in order to publish the beliefs about the states and the sensory prediction errors
  std_msgs::Float64MultiArray uAIC_mu, uAIC_mu_p, uAIC_mu_pp, SPE, torque_command, beliefs_mu_data, beliefs_mu_p_data;
  // Getting goal
  ros::Subscriber goal_mu_dSub;
};

#endif
