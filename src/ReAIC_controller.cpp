/*
 * File: ReAIC_controller.cpp

 * Author: Alon Dawe
 * 
 * Created: 13th October, 2023
 * 
 * Description: Class to perform Re-Active Inference Control of the 5-DOF Interbotix PincherX 150 robotic manipulator using ROS.
 * 
 * Original Author: Corrado Pezzato, TU Delft, DCSC 
 * (https://github.com/cpezzato/panda_simulation/blob/master/panda_control/src/AIC_controller.cpp)
 * The original author implemented an AIC controller to control a 7-DOF Franka Emika Panda robot arm.
 * The code in this file originated from this source, and was adapted to suit the control of a 5-DOF 
 * Interbotix PincherX 150 robotic manipulator with a different control algorithm. 
 */

#include "ReAIC.h"

// Constant for class ReAIC constructor to define which robot to control
const int robot = 1;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Reaic_control_node");
  // Variables to regulate the flow (Force to read once every 1ms the sensors)
  int count = 0;
  int cycles = 0;
  double t = 0.0;
  double sinusoidalPos = 0.0;
  const int joint = 3; 

  // joint 0 --> Waist
  // joint 1 --> Shoulder
  // joint 2 --> Elbow
  // joint 3 --> Wrist Angle
  // joint 4 --> Wrist Rotation

  //  Variables for sinusoidal trajectory
  const double frequency = 0.1;  // Frequency of the sinusoidal motion (adjust as needed)
  const double amplitude = 1.5;  // Amplitude of the sinusoidal motion (adjust as needed)


  // Variable for desired position (Goal Positions)
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

  // Object of the class ReAIC which will take care of everything
  ReAIC ReAIC_controller(robot);
  
  ros::Rate rate(1000);
  while (count<1000){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();
      count ++;
    rate.sleep();
  }

  // Set goal position in the ReAIC class
  ReAIC_controller.setGoal(desiredPos1);

  // Main loop
  while (ros::ok() && cycles <= 31000){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();

    // Skip only first cycle to allow reading the sensory input first
    if ((count!=0)&&(ReAIC_controller.dataReady()==1)){
      ReAIC_controller.minimiseF(0);
      cycles ++;

      if (cycles < 3000){
        ReAIC_controller.setGoal(desiredPos1);
      }
      // Step 1
      if (cycles >= 3000 && cycles < 8000){
        ReAIC_controller.setGoal(desiredPos2);
      }
      // Step 2
      if (cycles >= 8000 && cycles < 13000){
        ReAIC_controller.setGoal(desiredPos3);
      }
      // Step 3
      if (cycles >= 13000 && cycles < 18000){
        ReAIC_controller.setGoal(desiredPos1);
      }
      // Step 4
      if (cycles >= 18000 && cycles < 21000){
        ReAIC_controller.setGoal(desiredPos4);
      }
      // Step 5
      if (cycles >= 21000 && cycles < 24000){
        ReAIC_controller.setGoal(desiredPos5);
      }
      // Step 6
      if (cycles >= 24000 && cycles < 27000){
        ReAIC_controller.setGoal(desiredPos6);
      }
      // Step 7
      if (cycles >= 27000 && cycles < 31000){
        ReAIC_controller.setGoal(desiredPos1);
      }
    }
    else
      count ++;

    // Sinusoidal Trajectory Input
    //if ((count!=0)&&(ReAIC_controller.dataReady()==1)){
    //  ReAIC_controller.minimiseF();
    //  cycles ++;
    //  
    //  if (cycles <= 3000) {
    //  sinusoidalPos = amplitude * sin(2 * M_PI * frequency * t);
    //  desiredPos2[3] = sinusoidalPos;
    //  ReAIC_controller.setGoal(desiredPos2);
    //  } else {
    //    ReAIC_controller.setGoal(desiredPos1);
    //  }
//
    //  t = t + 0.01;
    //}
    //else
    //  count ++;
    
    rate.sleep();
  }
  ReAIC_controller.minimiseF(1);
  return 0;
}
