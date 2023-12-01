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

#include "AIC.h"

// Constant for class AIC constructor to define which robot to control
const int robot = 1;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aic_control_node");
  // Variables to regulate the flow (Force to read once every 1ms the sensors)
  int count = 0;
  int cycles = 0;
  double t = 0.0;
  double sinusoidalPos = 0.0;

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
  desiredPos2[0] = 0.8;

  desiredPos3 = desiredPos1;
  desiredPos3[0] = 1.6;

  desiredPos4 = desiredPos1;
  desiredPos4[0] = -0.2;

  desiredPos5 = desiredPos1;
  desiredPos5[0] = -0.4;

  desiredPos6 = desiredPos1;
  desiredPos6[0] = -0.6;

  //desiredPos2[0] = 0.8;
  //desiredPos2[0] = 0.0;
  //desiredPos2[1] = 0.0;
  //desiredPos2[2] = 0.0;
  //desiredPos2[3] = 0.0;
  //desiredPos2[3] = 0.8;
  //desiredPos2[4] = 0.0;
  //desiredPos2[4] = 0.8;
  

  //desiredPos3[0] = -0.8;
  //desiredPos3[0] = 0.0;
  //desiredPos3[1] = 0.0;
  //desiredPos3[2] = 0.0;
  //desiredPos3[3] = 0.0;
  //desiredPos3[3] = -0.8;
  //desiredPos3[4] = 0.0;
  //desiredPos3[4] = -0.8;
  //desiredPos3[5] = 0.0;

  // Complex Step Response
  //desiredPos3[0] = 0.0;
  //desiredPos3[0] = 1.6;
  //desiredPos3[1] = 0.0;
  //desiredPos3[2] = 0.0;
  //desiredPos3[3] = 0.0;
  //desiredPos3[3] = 1.6;
  //desiredPos3[4] = 0.0;
  //desiredPos3[4] = 1.6;

  //desiredPos4[0] = 0.0;
  //desiredPos4[0] = -0.2;
  //desiredPos4[1] = 0.0;
  //desiredPos4[2] = 0.0;
  //desiredPos4[3] = 0.0;
  //desiredPos4[3] = -0.2;
  //desiredPos4[4] = 0.0;
  //desiredPos4[4] = -0.2;

  //desiredPos5[0] = 0.0;
  //desiredPos5[0] = -0.4;
  //desiredPos5[1] = 0.0;
  //desiredPos5[2] = 0.0;
  //desiredPos5[3] = 0.0;
  //desiredPos5[3] = -0.4;
  //desiredPos5[4] = 0.0;
  //desiredPos5[4] = -0.4;

  //desiredPos6[0] = 0.0;
  //desiredPos6[0] = -0.6;
  //desiredPos6[1] = 0.0;
  //desiredPos6[2] = 0.0;
  //desiredPos6[3] = 0.0;
  //desiredPos6[3] = -0.6;
  //desiredPos6[4] = 0.0;
  //desiredPos6[4] = -0.6;

  // Object of the class AIC which will take care of everything
  AIC AIC_controller(robot);
  // Set desired position in the AIC class
  ros::Rate rate(100);
  while (count<100){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();
      count ++;
    rate.sleep();
  }

  AIC_controller.setGoal(desiredPos1);

  // Main loop
  
  while (ros::ok() && cycles <= 3100){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();

    // Skip only first cycle to allow reading the sensory input first
    if ((count!=0)&&(AIC_controller.dataReady()==1)){
      AIC_controller.minimiseF();
      cycles ++;

      if (cycles < 300){
        AIC_controller.setGoal(desiredPos1);
      }

      if (cycles >= 300 && cycles < 800){
        AIC_controller.setGoal(desiredPos2);
      }

      if (cycles >= 800 && cycles < 1300){
        AIC_controller.setGoal(desiredPos3);
      }

      if (cycles >= 1300 && cycles < 1800){
        AIC_controller.setGoal(desiredPos1);
      }

      if (cycles >= 1800 && cycles < 2100){
        AIC_controller.setGoal(desiredPos4);
      }

      if (cycles >= 2100 && cycles < 2400){
        AIC_controller.setGoal(desiredPos5);
      }

      if (cycles >= 2400 && cycles < 2700){
        AIC_controller.setGoal(desiredPos6);
      }

      if (cycles >= 2700 && cycles < 3100){
        AIC_controller.setGoal(desiredPos1);
      }

      //if (cycles < 600){
      //  AIC_controller.setGoal(desiredPos1);
      //}
//
      //if (cycles >= 600 && cycles < 1200){
      //  AIC_controller.setGoal(desiredPos2);
      //}
//
      //if (cycles >= 1200 && cycles < 1800){
      //  AIC_controller.setGoal(desiredPos1);
      //}
//
      //if (cycles >= 1800 && cycles < 2400){
      //  AIC_controller.setGoal(desiredPos3);
      //}
//
      //if (cycles >= 2400){
      //  AIC_controller.setGoal(desiredPos1);
      //  //cycles = 0;
      //}
    }
    else
      count ++;

    //if ((count!=0)&&(AIC_controller.dataReady()==1)){
    //  AIC_controller.minimiseF();
    //  cycles ++;
//
    //  if (cycles <= 3000) {
    //    sinusoidalPos = amplitude * sin(2 * M_PI * frequency * t);
    //    desiredPos2[3] = sinusoidalPos;
    //    AIC_controller.setGoal(desiredPos2);
    //  } else {
    //    AIC_controller.setGoal(desiredPos1);
    //  }
    //  t = t + 0.01;
    //}
    //else
    //  count ++;

    rate.sleep();
  }
  return 0;
}
