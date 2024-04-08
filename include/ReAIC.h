/*
 * File: ReAIC.h

 * Author: Alon Dawe
 * 
 * Created: 13th October, 2023
 * 
 * Description: Header file to perform Re-Active Inference Control of the 5-DOF Interbotix PincherX 150 robotic manipulator using ROS.
 * 
 * Original Author: Corrado Pezzato, TU Delft, DCSC 
 * (https://github.com/cpezzato/panda_simulation/blob/master/panda_control/include/AIC.h)
 * The original author implemented an AIC controller to control a 7-DOF Franka Emika Panda robot arm.
 * The code in this file originated from this source, and was adapted to suit the control of a 5-DOF 
 * Interbotix PincherX 150 robotic manipulator with a different control algorithm. 
 */

#ifndef ReAIC_H
#define ReAIC_H
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
#include <random>
#include <chrono>

// Class ReAIC to hanle the subscribers and the publishers for the Re-Active Inference Controller
class ReAIC
{
public:
  // Constructor and destructor
  ReAIC(int whichRobot);
  ~ReAIC();

  // Callback to handle the proprioceptive sensory data from the topic /joint_states published at 1kHz
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
  // Method to set the necessary variables once the constructor is called
  void initVariables();
  // Main method which minimises the free-energy using gradient descent
  void minimiseF(int stop);
  // Calculate and send the control commands to compute actions and further minimise the free-energy
  void computeActions(int stop);
  // Support method to control the program flow. Data ready returns one when the encoders has been read
  int dataReady();
  // Set desired position
  void setGoal(std::vector<double> desiredPos);
  // get methods for sensory prediction errors
  std_msgs::Float64MultiArray getSPE();
  // Gaussian noise function
  std::vector<double> generateNormalRandomNumbers(double mean, double stddev);
  // Function to stop controller once witin a certain error tolerance
  void adjust_learning_rate(); 

private:

  // Variances associated with the ReAIC and the confidence relative to sensory input and beliefs
  double var_q, var_qdot, var_mu, var_muprime, var_q_d, var_qdot_d;
  // Precision matrices, diagonal matrices with the inverce of the variance
  Eigen::Matrix<double, 5, 5> SigmaP_yq0, SigmaP_yq1, SigmaP_mu, SigmaP_muprime, SigmaP_yq0_d, SigmaP_yq1_d, k_a_adapt, k_mu_adapt, k_p_adapt;
  // Beliefs about the states and their derivatives mu, mu', mu'', column vectors of 5 elements
  Eigen::Matrix<double, 5, 1> mu, mu_p, mu_pp, mu_dot, mu_dot_p, mu_dot_pp, jointPos, jointVel;
  // Desired robot's states and error column vector of 5 elements
  Eigen::Matrix<double, 5, 1> mu_d, mu_p_d, error;
  // Control actions,  column vector of 5 elements
  Eigen::Matrix<double, 5, 1> u;
  // Learning rates and integration step for the ReAIC
  double k_mu, k_a, h, Kp, k_mu_original;
  // Sensory prediction errors
  double SPEq, SPEdq, SPEmu_p, SPEmu_pp, SPEq_d, SPEdq_d;
  // Support variable to control the flow of the script
  int dataReceived;
  // ROS related Variables, node handle
  ros::NodeHandle nh;
  // Publishers
  ros::Publisher singlePub, groupPub, tauPub1, tauPub2, tauPub3, tauPub4, tauPub5, tauPub6, tauPub7, IFE_pub;
  // Subscriber for proprioceptive sensors (i.e. from joint_states)
  ros::Subscriber sensorSub;
  // Support variables to contain the torques for the joints in gazebo
  std_msgs::Float64 tau1, tau2, tau3, tau4, tau5, tau6, tau7, F;
  // Values for direct kinematics computation using DH parameters (NOT USED)
  Eigen::Matrix<double, 7, 1> DH_a, DH_d, DH_alpha;
  Eigen::Matrix<double, 4, 4> DH_T, DH_A, T;
  Eigen::Matrix<double, 3, 1> eePosition;
  // Definition of variables in order to publish the beliefs about the states and the sensory prediction errors
  std_msgs::Float64MultiArray AIC_mu, AIC_mu_p, AIC_mu_pp, SPE, mu_des;
  // Publishers for beliefs
  ros::Publisher beliefs_mu_pub, beliefs_mu_p_pub, beliefs_mu_pp_pub, SPE_pub, mu_desired_pub;

  
};

#endif
