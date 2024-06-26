/*
 * File: AFC.cpp

 * Author: Alon Dawe
 * 
 * Created: 13th October, 2023
 * 
 * Description: Class to perform Adaptive Friction Control of the 5-DOF Interbotix PincherX 150 robotic manipulator using ROS.
 * 
 * Original Author: Corrado Pezzato, TU Delft, DCSC 
 * (https://github.com/cpezzato/panda_simulation/blob/master/panda_control/src/classes/AIC.cpp)
 * The original author implemented an AIC controller to control a 7-DOF Franka Emika Panda robot arm.
 * The code in this file originated from this source, and was adapted to suit the control of a 5-DOF 
 * Interbotix PincherX 150 robotic manipulator with a different control algorithm. 
 */

#include "AFC.h"

  // Constructor which takes as argument the publishers and initialises the private ones in the class
  AFC::AFC(int whichRobot){

      // Initialize publishers on the topics
      // Real robot publishers
      groupPub = nh.advertise<interbotix_xs_msgs::JointGroupCommand>("/px150/commands/joint_group", 20);
      singlePub = nh.advertise<interbotix_xs_msgs::JointSingleCommand>("/px150/commands/joint_single", 20);
      // Gazebo publishers
      tauPub1 = nh.advertise<std_msgs::Float64>("/px150/waist_controller/command", 20);
      tauPub2 = nh.advertise<std_msgs::Float64>("/px150/shoulder_controller/command", 20);
      tauPub3 = nh.advertise<std_msgs::Float64>("/px150/elbow_controller/command", 20);
      tauPub4 = nh.advertise<std_msgs::Float64>("/px150/wrist_angle_controller/command", 20);
      tauPub5 = nh.advertise<std_msgs::Float64>("/px150/wrist_rotate_controller/command", 20);
      //tauPub6 = nh.advertise<std_msgs::Float64>("/px150/Left_finger_controller/command", 20);
      //tauPub7 = nh.advertise<std_msgs::Float64>("/px150/right_finger_controller/command", 20);

      // Subscriber to the joint states
      sensorSub = nh.subscribe("/px150/joint_states", 1, &AFC::jointStatesCallback, this);
      
      // Publisher for the desired state
      mu_desired_pub = nh.advertise<std_msgs::Float64MultiArray>("mu_desired", 10);
    
    // Initialize the variables for the AFC
    AFC::initVariables();
  }
  AFC::~AFC(){}

  // Callback function for the subscriber
  void   AFC::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    //Gazebo Setup
    //jointPos(0) = msg->position[5];
    //jointVel(0) = msg->velocity[5];
    //jointPos(1) = msg->position[4];
    //jointVel(1) = msg->velocity[4];
    //jointPos(2) = msg->position[0];
    //jointVel(2) = msg->velocity[0];
    //jointPos(3) = msg->position[6];
    //jointVel(3) = msg->velocity[6];
    //jointPos(4) = msg->position[7];
    //jointVel(4) = msg->velocity[7];
    //jointPos(5) = msg->position[8];
    //jointVel(5) = msg->velocity[8];

    // Save joint sensor values
    for( int i = 0; i < 5; i++ ) {
      jointPos(i) = msg->position[i];
      jointVel(i) = msg->velocity[i];
    }
    // If this is the first time we read the joint states then we set the current beliefs
    if (dataReceived == 0){
      // Track the fact that the encoders published
      dataReceived = 1;
    }
  }

  // Function to initialize the variables for the AFC
  void AFC::initVariables(){

    // Support variable
    dataReceived = 0;

    // Tuning parameters are all initialised to zero
    Kp = Eigen::Matrix<double, 5, 5>::Zero();
    Kd = Eigen::Matrix<double, 5, 5>::Zero();
    Ki = Eigen::Matrix<double, 5, 5>::Zero();
    P = Eigen::Matrix<double, 5, 5>::Zero();
    lambda = Eigen::Matrix<double, 5, 5>::Zero();
  
    // Begin Tuning parameters of AFC (Read from ./config/AFC_tuning.yaml)
    //---------------------------------------------------------------
    
    // Variances associated with the beliefs and the sensory inputs
    ROS_INFO("Setting AFC parameters from parameter space");
    nh.getParam("k_p", k_p);
    nh.getParam("k_d", k_d);
    nh.getParam("k_i", k_i);
    nh.getParam("p", p);
    nh.getParam("lam", lam);

    for( int i = 0; i < Kp.rows(); i = i + 1 ) {
      Kp(i,i) = k_p;
      Kd(i,i) = k_d;
      Ki(i,i) = k_i;
      P(i,i) = p;
      lambda(i,i) = lam;
    }

    // Initialize control actions
    u << 0.0, 0.0, 0.0, 0.0, 0.0;
    u_AFC << 0.0, 0.0, 0.0, 0.0, 0.0;

    error << 0.0, 0.0, 0.0, 0.0, 0.0;
    error_p << 0.0, 0.0, 0.0, 0.0, 0.0;

    k_c_dot << 0.0, 0.0, 0.0, 0.0, 0.0;
    k_c_dot_prev << 0.0, 0.0, 0.0, 0.0, 0.0;
    k_c << 0.0, 0.0, 0.0, 0.0, 0.0;
    k_c_prev << 0.0, 0.0, 0.0, 0.0, 0.0;
    sigma << 0.0, 0.0, 0.0, 0.0, 0.0;

    gravity_flag << 0.0, 0.0, 0.0, 0.0, 0.0;

    mu_d << 0.0, 0.0, 0.0, 0.0, 0.0;

    mu_p_d << 0.0, 0.0, 0.0, 0.0, 0.0;

    se << 0.0, 0.0, 0.0, 0.0, 0.0;

    // Integration step
    h = 0.001;

    mu_des.data.resize(5);

  }

  // SIGNUM function Original Article format --> No gravity compensation
  //void AFC::signum_1(){
  //  for(int i =0; i<jointVel.rows(); i = i + 1){
  //    if(abs(error(i)) < 0.01){
  //      sigma(i) = 0;
  //    } else if(jointVel(i) > 0.0 || (jointVel(i) == 0 && u(i) > 0.0)){
  //      sigma(i) = 1;
  //    } else if(jointVel(i) < 0.0 || (jointVel(i) == 0 && u(i) < 0.0)){
  //      sigma(i) = -1;
  //    } else{
  //      sigma(i) = 0;
  //    }
  //  }
  //}

  // SIGNUM function Modified for gravity compensation
  void AFC::signum_1(){
    for(int i =0; i<jointVel.rows(); i = i + 1){
      if(abs(error(i)) < 0.01){
        sigma(i) = 0;
      } else if(jointVel(i) > 0.0 || (jointVel(i) == 0 && u(i) > Ki(i,i) * abs(se(i)))){
        sigma(i) = 1;
      } else if(jointVel(i) < 0.0 || (jointVel(i) == 0 && u(i) < - Ki(i,i) * abs(se(i)))){
        sigma(i) = -1;
      } else{
        sigma(i) = 0;
      }
    }
  }

  // Compute control actions
  void   AFC::computeActions(){
    //AFC::adjust_learning_rate();
    
    error = (mu_d - jointPos);
    for( int i = 0; i < error.rows(); i = i + 1 ) {
      if(abs(error(i)) < 0.01){
          se(i) = se(i);
      }else if (abs(error(i)) < 0.5){
          se(i) = se(i) + error(i)*h;
      }
    }

    // Prevent integral wind-up
    for( int i = 0; i < se.rows(); i = i + 1 ) {
      if (se(i) > 0.5) {
        se(i) = 0.5;
      }
      else if(se(i) < -0.5){
        se(i) = -0.5;
      }
    }

    error_p = (mu_p_d - jointVel);

    // Compute control actions
    u = 590* (Kp * error + Kd * error_p + Ki * se);
    //u = 590* (Kp * error + Kd * error_p);

    // Friction compensation
    AFC::signum_1();
    k_c_dot = (P * sigma).cwiseProduct((error_p + lambda*error));
    k_c = k_c_prev + h/2 * k_c_dot_prev + h/2 * k_c_dot; 
    k_c_dot_prev = k_c_dot;
    k_c_prev = k_c; 
    
    // Update control action to include friction compensation
    u = u + 590*k_c.cwiseProduct(sigma);

    // Prevent control action windup
    for( int i = 0; i < u.rows(); i = i + 1 ) {
      if (u(i) > 885.0 ){
        u(i) = 885.0;
      }
      else if (u(i) < -885.0){
        u(i) = -885.0;
      }
    }

    interbotix_xs_msgs::JointGroupCommand a;

    interbotix_xs_msgs::JointSingleCommand waist_msg;
    interbotix_xs_msgs::JointSingleCommand shoulder_msg;
    interbotix_xs_msgs::JointSingleCommand elbow_msg;
    interbotix_xs_msgs::JointSingleCommand wrist_ang_msg;
    interbotix_xs_msgs::JointSingleCommand wrist_rot_msg;


    a.name = "arm";

    waist_msg.name = "waist";
    waist_msg.cmd = u(0);

    shoulder_msg.name = "shoulder";
    shoulder_msg.cmd = u(1);

    elbow_msg.name = "elbow";
    elbow_msg.cmd = u(2);

    wrist_ang_msg.name = "wrist_angle";
    wrist_ang_msg.cmd = u(3);

    wrist_rot_msg.name = "wrist_rotate";
    wrist_rot_msg.cmd = u(4);

    //singlePub.publish(waist_msg);
    //singlePub.publish(elbow_msg);
    //singlePub.publish(wrist_ang_msg);
    singlePub.publish(wrist_rot_msg);

    // Publish the message.
    //groupPub.publish(a);
    

    // Set the toques from u and publish for gazebo setup
    tau1.data = u(0); tau2.data = u(1); tau3.data = u(2); tau4.data = u(3);
    tau5.data = u(4); //tau6.data = u(5); //tau7.data = u(6);
    // Publishing
    tauPub1.publish(tau1); tauPub2.publish(tau2); tauPub3.publish(tau3);
    tauPub4.publish(tau4); //tauPub5.publish(tau5); //tauPub6.publish(tau6);
    //tauPub7.publish(tau7);
  }

  // Method to control if the joint states have been received already,
  // used in the main function
  int AFC::dataReady(){
    if(dataReceived==1)
      return 1;
    else
      return 0;
  }

  // function to publish the goal states
  void AFC::setGoal(std::vector<double> desiredPos){
    for(int i=0; i<desiredPos.size(); i++){
      mu_d(i) = desiredPos[i];
      mu_des.data[i] = mu_d(i);
    }
    mu_desired_pub.publish(mu_des);
  }

  // function for the impulse step response
  void AFC::setStep(std::vector<double> controlInput){
    for(int i=0; i<controlInput.size(); i++){
      u(i) = 300*(controlInput[i]);
    }

    interbotix_xs_msgs::JointGroupCommand a;

    interbotix_xs_msgs::JointSingleCommand waist_msg;
    interbotix_xs_msgs::JointSingleCommand shoulder_msg;
    interbotix_xs_msgs::JointSingleCommand elbow_msg;
    interbotix_xs_msgs::JointSingleCommand wrist_ang_msg;
    interbotix_xs_msgs::JointSingleCommand wrist_rot_msg;


    a.name = "arm";

    waist_msg.name = "waist";
    waist_msg.cmd = u(0);

    shoulder_msg.name = "shoulder";
    shoulder_msg.cmd = u(1);

    elbow_msg.name = "elbow";
    elbow_msg.cmd = u(2);

    wrist_ang_msg.name = "wrist_angle";
    wrist_ang_msg.cmd = u(3);

    wrist_rot_msg.name = "wrist_rotate";
    wrist_rot_msg.cmd = u(4);

    singlePub.publish(waist_msg);
    //singlePub.publish(elbow_msg);
    //singlePub.publish(wrist_ang_msg);
    //singlePub.publish(wrist_rot_msg);

    // Set the toques from u and publish
    tau1.data = u(0); tau2.data = u(1); tau3.data = u(2); tau4.data = u(3);
    tau5.data = u(4); //tau6.data = u(5); //tau7.data = u(6);
    // Publishing
    tauPub1.publish(tau1); tauPub2.publish(tau2); tauPub3.publish(tau3);
    tauPub4.publish(tau4); //tauPub5.publish(tau5); //tauPub6.publish(tau6);
    //tauPub7.publish(tau7);
  }
