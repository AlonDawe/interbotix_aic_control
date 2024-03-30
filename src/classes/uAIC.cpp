/*
 * File: uAIC.cpp

 * Author: Alon Dawe
 * 
 * Created: 13th October, 2023
 * 
 * Description: Class to perform Unbiased Active Inference Control of the 5-DOF Interbotix PincherX 150 robotic manipulator using ROS.
 * 
 * Original Author: Corrado Pezzato, TU Delft, DCSC 
 * (https://github.com/cpezzato/unbiased_aic/blob/master/src/classes/uAIC.cpp)
 * The original author implemented an ubiased AIC controller to control a 7-DOF Franka Emika Panda robot arm.
 * The code in this file originated from this source, and was adapted to suit the control of a 5-DOF 
 * Interbotix PincherX 150 robotic manipulator
 */

#include "uAIC.h"

  // Constructor which takes as argument the publishers and initialises the private ones in the class
  uAIC::uAIC(int whichRobot){
    // Initialize the variables for thr uAIC
    uAIC::initVariables();
      // Torque publisher
      groupPub = nh.advertise<interbotix_xs_msgs::JointGroupCommand>("/px150/commands/joint_group", 20);
      singlePub = nh.advertise<interbotix_xs_msgs::JointSingleCommand>("/px150/commands/joint_single", 20);

      tauPub1 = nh.advertise<std_msgs::Float64>("/px150/waist_controller/command", 20);
      tauPub2 = nh.advertise<std_msgs::Float64>("/px150/shoulder_controller/command", 20);
      tauPub3 = nh.advertise<std_msgs::Float64>("/px150/elbow_controller/command", 20);
      tauPub4 = nh.advertise<std_msgs::Float64>("/px150/wrist_angle_controller/command", 20);
      tauPub5 = nh.advertise<std_msgs::Float64>("/px150/wrist_rotate_controller/command", 20);
      //tauPub6 = nh.advertise<std_msgs::Float64>("/px150/Left_finger_controller/command", 20);
      //tauPub7 = nh.advertise<std_msgs::Float64>("/px150/right_finger_controller/command", 20);

      //Subscriber
      sensorSub = nh.subscribe("/px150/joint_states", 1, &uAIC::jointStatesCallback, this);

      // Publisher beliefs
      beliefs_mu_pub = nh.advertise<std_msgs::Float64MultiArray>("/beliefs_mu", 20);
      beliefs_mu_p_pub = nh.advertise<std_msgs::Float64MultiArray>("/beliefs_mu_p", 20);

      // Listener to goals
      goal_mu_dSub = nh.subscribe("/desired_state", 5, &uAIC::setDesiredState, this);

  }
  uAIC::~uAIC(){}

  // Method to set the current goal from topic, this is the input to the controller
  void uAIC::setDesiredState(const sensor_msgs::JointState::ConstPtr& msg){
    for( int i = 0; i < 5; i++ ) {
      //mu_d(i) = msg->ref_position.data[i];
      mu_d(i) = jointPos(i);
      mu_p_d(i) = msg->velocity[i];
      // mu_p_d(i) = 0.0;
    }
  }

  void   uAIC::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
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

    // Save joint values
    for( int i = 0; i < 5; i++ ) {
      jointPos(i) = msg->position[i];
      jointVel(i) = msg->velocity[i];
    }
    // If this is the first time we read the joint states then we set the current beliefs
    if (dataReceived == 0){
      // Track the fact that the encoders published
      dataReceived = 1;
      // The first time we retrieve the position we define the initial beliefs about the states
      mu = jointPos;
      mu_p = jointVel;
      mu_past = mu;
      mu_p_past = mu_p;
    }
  }

  void   uAIC::initVariables(){

    // Support variable
    dataReceived = 0;

    // Precision matrices (first set them to zero then populate the diagonal)
    SigmaP_yq0 = Eigen::Matrix<double, 5, 5>::Zero();
    SigmaP_yq1 = Eigen::Matrix<double, 5, 5>::Zero();
    SigmaP_mu = Eigen::Matrix<double, 5, 5>::Zero();
    SigmaP_muprime = Eigen::Matrix<double, 5, 5>::Zero();
    K_p = Eigen::Matrix<double, 5, 5>::Zero();
    K_d = Eigen::Matrix<double, 5, 5>::Zero();
    K_i = Eigen::Matrix<double, 5, 5>::Zero();

    // Begin Tuning parameters of u-AIC
    //---------------------------------------------------------------

    // Variances associated with the beliefs and the sensory inputs
    ROS_INFO("Setting u-AIC parameters from parameter space");
    nh.getParam("var_mu", var_mu);
    nh.getParam("var_muprime", var_muprime);
    nh.getParam("var_q", var_q);
    nh.getParam("var_qdot", var_qdot);

    // Controller values, diagonal elements of the gain matrices for the PID like control law
    nh.getParam("k_p0", k_p0);
    nh.getParam("k_p1", k_p1);
    nh.getParam("k_p2", k_p2);
    nh.getParam("k_p3", k_p3);
    nh.getParam("k_p4", k_p4);
    nh.getParam("k_d", k_d);
    nh.getParam("k_i", k_i);
    nh.getParam("k_mu", k_mu);
    nh.getParam("k_a", k_a);
    nh.getParam("max_i", max_i);
    I_gain <<  0.02, 0.02, 0.02, 0.02, 0.02;

    // Learning rates for the gradient descent (found that a ratio of 60 works good)

    // End tuning parameters
    //---------------------------------------------------------------

    // Populate matrices
    for( int i = 0; i < SigmaP_yq0.rows(); i = i + 1 ) {
        SigmaP_yq0(i,i) = 1/var_q;
        SigmaP_yq1(i,i) = 1/var_qdot;
        SigmaP_mu(i,i) = 1/var_mu;
        SigmaP_muprime(i,i) = 1/var_muprime;
        //K_p(i,i) = k_p;
        K_d(i,i) = k_d;
        K_i(i,i) = k_i;
    }

    // Single proportional
    K_p(0,0) = k_p0;
    K_p(1,1) = k_p1; 
    K_p(2,2) = k_p2;
    K_p(3,3) = k_p3;
    K_p(4,4) = k_p4;

    // Initialize control actions
    u << 0.0, 0.0, 0.0, 0.0, 0.0;

    // Initialize prior beliefs about the second ordet derivatives of the states of the robot
    mu_pp << 0.0, 0.0, 0.0, 0.0, 0.0;

    // Integration step
    h = 0.001;
    // Resize the data for the published message
    torque_command.data.resize(5);
    beliefs_mu_data.data.resize(5);
    beliefs_mu_p_data.data.resize(5);
  }

  void uAIC::minimiseF(){
  // Unbiased uAIC
    for (int i=0;i<1;i++){
      mu_dot = - k_mu*(-SigmaP_yq0*(jointPos-mu) + SigmaP_mu*(mu - (mu_past + h*mu_p_past)));
      mu_dot_p = - k_mu*(-SigmaP_yq1*(jointVel-mu_p) + SigmaP_muprime*(mu_p-mu_p_past));

      // Save current value of the belief to use in the next iteration as previous value
      mu_past = mu;
      mu_p_past = mu_p;

      // Belifs update
      mu = mu + h*mu_dot;             // Belief about the position
      mu_p = mu_p + h*mu_dot_p;       // Belief about motion of mu
      //mu_pp = mu_pp + h*mu_dot_pp;    // Belief about motion of mu'
    }
    // Set curret values for next ieration
    I_gain = I_gain + mu_d-mu;
    // Satruration of integral action
    for(int j = 0; j<5; j++){
      if(I_gain(j)>max_i){
          I_gain(j) = max_i;
      }
      if(I_gain(j)<-max_i){
          I_gain(j) = -max_i;
      }
    }
	//ROS_WARN("Current integral term: %f",I_gain(0));
        // Calculate and send control actions
    uAIC::computeActions();
    
    for (int i=0;i<5;i++){
    	beliefs_mu_data.data[i] = mu(i);
    	beliefs_mu_p_data.data[i] = mu_p(i);
    }

    beliefs_mu_pub.publish(beliefs_mu_data);
    beliefs_mu_p_pub.publish(beliefs_mu_p_data);
  }

  void   uAIC::computeActions(){
    // Unbiased uAIC
    u = K_p*(mu_d-mu) + K_d*(mu_p_d-mu_p) + K_i*(I_gain);

    interbotix_xs_msgs::JointGroupCommand a;

    a.name = "arm";
    a.cmd.push_back(u(0));
    a.cmd.push_back(u(1));
    a.cmd.push_back(u(2));
    a.cmd.push_back(u(3));
    a.cmd.push_back(u(4));
    a.cmd.push_back(0.0);

    groupPub.publish(a);

    // Set the toques from u and publish
    tau1.data = u(0); tau2.data = u(1); tau3.data = u(2); tau4.data = u(3);
    tau5.data = u(4);
    // Publishing
    tauPub1.publish(tau1); tauPub2.publish(tau2); tauPub3.publish(tau3);
    tauPub4.publish(tau4); tauPub5.publish(tau5); 
  }

  // Method to control if the joint states have been received already,
  // used in the main function
  int uAIC::dataReady(){
    if(dataReceived==1)
      return 1;
    else
      return 0;
  }

  // Method to set the desired position from script, used in the main to initialize the arm to a central pose and keep it there
  void uAIC::setGoal(std::vector<double> desiredPos){
    for(int i=0; i<desiredPos.size(); i++){
      mu_d(i) = desiredPos[i];
      mu_p_d(i) = 0;
    }
  }

  void uAIC::setGoalCurrentState(){
    for(int i=0; i<5; i++){
      mu_d(i) = jointPos(i);
      mu_p_d(i) = 0;
//      std::cout << jointPos(i) << "\n";
    }
  }

  std_msgs::Float64MultiArray  uAIC::getSPE(){
    return(SPE);
  }
