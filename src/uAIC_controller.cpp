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

#include "uAIC.h"

// Constant for class AIC constructor to define which robot to control
const int robot = 1;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uaic_control_node");
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

  desiredPos2[0] = 1.5708;
  desiredPos2[1] = -0.3;
  desiredPos2[2] = 0.3;
  desiredPos2[3] = 0.0;
  desiredPos2[4] = 0.0;

  desiredPos3[0] = 2*1.5708;
  desiredPos3[1] = -0.6;
  desiredPos3[2] = 0.6;
  desiredPos3[3] = 0.0;
  desiredPos3[4] = 0.0;

  // Object of the class AIC which will take care of everything
  uAIC uAIC_controller(robot);
  // Set desired position in the AIC class
  ros::Rate rate(1000);
  while (count<1000){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();
      count ++;
    rate.sleep();
  }

  uAIC_controller.setGoal(desiredPos1);

  // Main loop
  
  while (ros::ok()){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();

    // Skip only first cycle to allow reading the sensory input first
    if ((count!=0)&&(uAIC_controller.dataReady()==1)){
      uAIC_controller.minimiseF();
      cycles ++;
      if (cycles == 6000){
        uAIC_controller.setGoal(desiredPos2);
      }

      if (cycles == 12000){
        uAIC_controller.setGoal(desiredPos3);
      }

      if (cycles == 18000){
        uAIC_controller.setGoal(desiredPos2);
      }

      if (cycles == 24000){
        uAIC_controller.setGoal(desiredPos1);
        cycles = 0;
      }
    }
    else
      count ++;

    rate.sleep();
  }
  return 0;
}
