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
  double t = 0.0;
  double sinusoidalPos = 0.0;
  const int joint = 0;

  //  Variables for sinusoidal trajectory
  const double frequency = 0.1;  // Frequency of the sinusoidal motion (adjust as needed)
  const double amplitude = 1.5;  // Amplitude of the sinusoidal motion (adjust as needed)



  // Variable for desired position, set here the goal for the Panda for each joint
  std::vector<double> desiredPos1(5), desiredPos2(5), desiredPos3(5), desiredPos4(5), desiredPos5(5), desiredPos6(5);

  desiredPos1[0] = 0.0;
  desiredPos1[1] = 0.0;
  desiredPos1[2] = 0.0;
  desiredPos1[3] = 0.0;
  desiredPos1[4] = 0.0;

  desiredPos2 = desiredPos1;
  desiredPos2[joint] = 0.8;

  desiredPos3 = desiredPos1;
  desiredPos3[joint] = 1.6;

  desiredPos4 = desiredPos1;
  desiredPos4[joint] = -0.2;

  desiredPos5 = desiredPos1;
  desiredPos5[joint] = -0.4;

  desiredPos6 = desiredPos1;
  desiredPos6[joint] = -0.6;



  // Object of the class PID which will take care of everything
  PID PID_controller(robot);
  // Set desired position in the PID class
  ros::Rate rate(1000);
  while (count<1000){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();
      count ++;
    rate.sleep();
  }

  PID_controller.setGoal(desiredPos1);

  // Main loop
  
  while (ros::ok() && cycles <= 31000){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();

    // Skip only first cycle to allow reading the sensory input first
    if ((count!=0)&&(PID_controller.dataReady()==1)){
      PID_controller.computeActions();
      cycles ++;

      if (cycles < 3000){
        PID_controller.setGoal(desiredPos1);
      }

      if (cycles >= 3000 && cycles < 8000){
        PID_controller.setGoal(desiredPos2);
      }

      if (cycles >= 8000 && cycles < 13000){
        PID_controller.setGoal(desiredPos3);
      }

      if (cycles >= 13000 && cycles < 18000){
        PID_controller.setGoal(desiredPos1);
      }

      if (cycles >= 18000 && cycles < 21000){
        PID_controller.setGoal(desiredPos4);
      }

      if (cycles >= 21000 && cycles < 24000){
        PID_controller.setGoal(desiredPos5);
      }

      if (cycles >= 24000 && cycles < 27000){
        PID_controller.setGoal(desiredPos6);
      }

      if (cycles >= 27000 && cycles < 31000){
        PID_controller.setGoal(desiredPos1);
      }


      //if (cycles < 600){
      //  PID_controller.setGoal(desiredPos1);
      //}
//
      //if (cycles >= 600 && cycles < 1200){
      //  PID_controller.setGoal(desiredPos2);
      //}
//
      //if (cycles >= 1200 && cycles < 1800){
      //  PID_controller.setGoal(desiredPos1);
      //}
//
      //if (cycles >= 1800 && cycles < 2400){
      //  PID_controller.setGoal(desiredPos3);
      //}
//
      //if (cycles >= 2400){
      //  PID_controller.setGoal(desiredPos1);
      //  //cycles = 0;
      //}
    }
    else
      count ++;

    //if ((count!=0)&&(PID_controller.dataReady()==1)){
    //  PID_controller.computeActions();
    //  cycles ++;
    //  
    //  if (cycles <= 3000) {
    //  sinusoidalPos = amplitude * sin(2 * M_PI * frequency * t);
    //  desiredPos2[3] = sinusoidalPos;
    //  PID_controller.setGoal(desiredPos2);
    //  } else {
    //    PID_controller.setGoal(desiredPos1);
    //  }
//
    //  t = t + 0.01;
    //}
    //else
    //  count ++;

    rate.sleep();
  }
  return 0;
}
