/*
 * File:   AIC_controller.cpp
 * Author: Corrado Pezzato, TU Delft, DCSC
 *
 * Created on April 14th, 2019
 *
 * This node allows to control the 7DOF Franka Emika Panda robot arm through
 * the new promisin theory called Active Inference proposed by Karl Friston.
 *
 * The robot moves to the desired position specified in desiredPos performing
 * free-energy minimization and actiove inference using gradient descent.
 * The robot is equipped with proprioceptive sensors for joints position and
 * velocity and a camera for the end-effector pose estimation.
 * The control is in joint space.
 *
 */

#include "PID.h"

// Constant for class PID constructor to define which robot to control
const int robot = 1;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid_control_node");
  // Variables to regulate the flow (Force to read once every 1ms the sensors)
  int count = 0;
  int cycles = 0;
  // Variable for desired position, set here the goal for the Panda for each joint
  std::vector<double> desiredPos1(5), desiredPos2(5), desiredPos3(5);

  desiredPos1[0] = 0.0;
  desiredPos1[1] = 0.0;
  desiredPos1[2] = 0.0;
  desiredPos1[3] = 0.0;
  desiredPos1[4] = 0.0;
  //desiredPos1[5] = 0.0;

  desiredPos2[0] = 1.5708;
  //desiredPos2[0] = 0.0;
  desiredPos2[1] = 0.0;
  desiredPos2[2] = 0.0;
  desiredPos2[3] = 0.0;
  desiredPos2[4] = 0.0;
  //desiredPos2[5] = 0.0;

  desiredPos3[0] = -1.5708;
  //desiredPos3[0] = 0.0;
  desiredPos3[1] = 0.0;
  desiredPos3[2] = 0.0;
  desiredPos3[3] = 0.0;
  desiredPos3[4] = 0.0;
  //desiredPos3[5] = 0.0;

  // Object of the class PID which will take care of everything
  PID PID_controller(robot);
  // Set desired position in the PID class
  ros::Rate rate(100);
  while (count<100){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();
      count ++;
    rate.sleep();
  }

  PID_controller.setGoal(desiredPos1);

  // Main loop
  
  while (ros::ok()){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();

    // Skip only first cycle to allow reading the sensory input first
    if ((count!=0)&&(PID_controller.dataReady()==1)){
      PID_controller.computeActions();
      cycles ++;
      if (cycles == 600){
        PID_controller.setGoal(desiredPos2);
      }

      if (cycles == 1200){
        PID_controller.setGoal(desiredPos1);
      }

      if (cycles == 1800){
        PID_controller.setGoal(desiredPos3);
      }

      if (cycles == 2400){
        PID_controller.setGoal(desiredPos1);
        cycles = 0;
      }
    }
    else
      count ++;

    rate.sleep();
  }
  return 0;
}
