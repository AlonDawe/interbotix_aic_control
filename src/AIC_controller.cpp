/*
 * File: AIC_controller.cpp

 * Author: Alon Dawe
 * 
 * Created: 13th October, 2023
 * 
 * Description: Class to perform Active Inference Control of the 5-DOF Interbotix PincherX 150 robotic manipulator using ROS.
 * 
 * Original Author: Corrado Pezzato, TU Delft, DCSC 
 * (https://github.com/cpezzato/panda_simulation/blob/master/panda_control/src/AIC_controller.cpp)
 * The original author implemented an AIC controller to control a 7-DOF Franka Emika Panda robot arm.
 * The code in this file originated from this source, and was adapted to suit the control of a 5-DOF 
 * Interbotix PincherX 150 robotic manipulator with a different control algorithm. 
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
  const int joint = 3;

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
  ros::Rate rate(1000);
  while (count<1000){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();
      count ++;
    rate.sleep();
  }

  AIC_controller.setGoal(desiredPos1);

  //ros::Time last_time = ros::Time::now();

  // Main loop
  
  while (ros::ok() && cycles <= 31000){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();

    // Skip only first cycle to allow reading the sensory input first
    if ((count!=0)&&(AIC_controller.dataReady()==1)){
      AIC_controller.minimiseF();
      cycles ++;

      if (cycles < 3000){
        AIC_controller.setGoal(desiredPos1);
      }

      if (cycles >= 3000 && cycles < 8000){
        AIC_controller.setGoal(desiredPos2);
      }

      if (cycles >= 8000 && cycles < 13000){
        AIC_controller.setGoal(desiredPos3);
      }

      if (cycles >= 13000 && cycles < 18000){
        AIC_controller.setGoal(desiredPos1);
      }

      if (cycles >= 18000 && cycles < 21000){
        AIC_controller.setGoal(desiredPos4);
      }

      if (cycles >= 21000 && cycles < 24000){
        AIC_controller.setGoal(desiredPos5);
      }

      if (cycles >= 24000 && cycles < 27000){
        AIC_controller.setGoal(desiredPos6);
      }

      if (cycles >= 27000 && cycles < 31000){
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
    //ros::Time current_time = ros::Time::now();
    //double loop_duration = (current_time - last_time).toSec();
    //last_time = current_time;

    //ROS_WARN("[ INFO] [%.9f]: Loop duration", loop_duration);
    
    rate.sleep();
  }
  return 0;
}
