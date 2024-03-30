/*
 * File: OL_step_response.cpp
 *
 * Author: Alon Dawe
 * 
 * Created: 13th October, 2023
 * 
 * Description: Class to perform an open-loop  step response of the 5-DOF Interbotix PincherX 150 robotic manipulator using ROS.
 * 
 */

#include "PID.h"

// Constant for class AIC constructor to define which robot to control
const int robot = 1;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "OL_STEP");
  // Variables to regulate the flow (Force to read once every 1ms the sensors)
  int count = 0;
  int cycles = 0;
  // Variable for desired position, set here the goal for the Panda for each joint
  std::vector<double> controlInput1(5), controlInput2(5);

  controlInput1[0] = 1.0;
  controlInput1[1] = 0.0;
  controlInput1[2] = 0.0;
  controlInput1[3] = 0.0;
  controlInput1[4] = 0.0;

  controlInput2[0] = 0.0;
  controlInput2[1] = 0.0;
  controlInput2[2] = 0.0;
  controlInput2[3] = 0.0;
  controlInput2[4] = 0.0;

  // Object of the class AIC which will take care of everything
  PID PID_controller(robot);
  // Set desired position in the AIC class
  ros::Rate rate(1000);
  while (count<1000){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();
      count ++;
    rate.sleep();
  }

  //AIC_controller.setGoal(desiredPos1);

  // Main loop
  PID_controller.setStep(controlInput2);
  while (ros::ok() && cycles <= 400){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();

    // Skip only first cycle to allow reading the sensory input first
    if ((count!=0)&&(PID_controller.dataReady()==1)){
      cycles ++;
      if (cycles < 400){
        PID_controller.setStep(controlInput1);
      } else {
        PID_controller.setStep(controlInput2);
      }
    }
    else
      count ++;

    rate.sleep();
  }

  return 0;
}
