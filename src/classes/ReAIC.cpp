/*
 * File: ReAIC.cpp

 * Author: Alon Dawe
 * 
 * Created: 13th October, 2023
 * 
 * Description: Class to perform Re-Active Inference Control of the 5-DOF Interbotix PincherX 150 robotic manipulator using ROS.
 * 
 * Original Author: Corrado Pezzato, TU Delft, DCSC 
 * (https://github.com/cpezzato/panda_simulation/blob/master/panda_control/src/classes/AIC.cpp)
 * The original author implemented an AIC controller to control a 7-DOF Franka Emika Panda robot arm.
 * The code in this file originated from this source, and was adapted to suit the control of a 5-DOF 
 * Interbotix PincherX 150 robotic manipulator with a different control algorithm. 
 */

#include "ReAIC.h"

  // Constructor which takes as argument the publishers and initialises the private ones in the class
  ReAIC::ReAIC(int whichRobot){

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
      sensorSub = nh.subscribe("/px150/joint_states", 1, &ReAIC::jointStatesCallback, this);

      // Publisher for the free-energy and sensory prediction errors
      IFE_pub = nh.advertise<std_msgs::Float64>("px150_free_energy", 10);
      SPE_pub = nh.advertise<std_msgs::Float64MultiArray>("px150_SPE", 10);

      // Publishers for beliefs
      beliefs_mu_pub = nh.advertise<std_msgs::Float64MultiArray>("beliefs_mu", 10);
      beliefs_mu_p_pub = nh.advertise<std_msgs::Float64MultiArray>("beliefs_mu_p", 10);
      beliefs_mu_pp_pub = nh.advertise<std_msgs::Float64MultiArray>("beliefs_mu_pp", 10);

      // Publisher for the desired state
      mu_desired_pub = nh.advertise<std_msgs::Float64MultiArray>("mu_desired", 10);

    // Initialize the variables for the ReAIC
    ReAIC::initVariables();
  }
  ReAIC::~ReAIC(){}

  // Function to generate Gaussian Noise for each joint
  std::vector<double> ReAIC::generateNormalRandomNumbers(double mean, double stddev) {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count(); // Seed using system time
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(mean, stddev);
    std::vector<double> result(5);

    for (int i = 0; i < 5; ++i) {
        result[i] = distribution(generator);
    }
    
    return result;
  }

  // Callback function for the subscriber
  void   ReAIC::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    //Gazebo Setup
    //jointPos(0) = msg->position[6];
    //jointVel(0) = msg->velocity[6];
    //jointPos(1) = msg->position[5];
    //jointVel(1) = msg->velocity[5];
    //jointPos(2) = msg->position[0];
    //jointVel(2) = msg->velocity[0];
    //jointPos(3) = msg->position[1];
    //jointVel(3) = msg->velocity[1];
    //jointPos(4) = msg->position[7];
    //jointVel(4) = msg->velocity[7];
    //jointPos(5) = msg->position[8];
    //jointVel(5) = msg->velocity[8];


    // Simulate gaussian noise on the sensors
    //std::vector<double> noise = ReAIC::generateNormalRandomNumbers(0.0, 0.1);

    // Save joint sensor values
    for( int i = 0; i < 5; i++ ) {
      jointPos(i) = msg->position[i];// + noise[i];
      jointVel(i) = msg->velocity[i];// + noise[i];
    }

    // If this is the first time we read the joint states then we set the current beliefs
    if (dataReceived == 0){
      // Track the fact that the encoders published
      dataReceived = 1;
      // The first time we retrieve the position we define the initial beliefs about the states
      mu = jointPos;
      mu_p = jointVel;
    }
  }

  // Function to initialize the variables for the ReAIC
  void   ReAIC::initVariables(){

    // Support variable
    dataReceived = 0;

    // Precision matrices (first set them to zero then populate the diagonal)
    // Variances associated with the sensory inputs
    SigmaP_yq0 = Eigen::Matrix<double, 5, 5>::Zero();
    SigmaP_yq1 = Eigen::Matrix<double, 5, 5>::Zero();

    // Variances associated with the beliefs
    SigmaP_mu = Eigen::Matrix<double, 5, 5>::Zero();
    SigmaP_muprime = Eigen::Matrix<double, 5, 5>::Zero();

    // Learning rates for the gradient descent
    k_a_adapt = Eigen::Matrix<double, 5, 5>::Zero();
    k_mu_adapt = Eigen::Matrix<double, 5, 5>::Zero();

    // Proportional tuning parameter for the ReAIC
    k_p_adapt = Eigen::Matrix<double, 5, 5>::Zero();

    // Begin tuning parameters of ReAIC (Read from ./config/ReAIC_tuning.yaml)
    //---------------------------------------------------------------

    // Variances associated with the beliefs and the sensory inputs
    ROS_INFO("Setting ReAIC parameters from parameter space");
    nh.getParam("var_mu", var_mu);
    nh.getParam("var_muprime", var_muprime);
    nh.getParam("var_q", var_q);
    nh.getParam("var_qdot", var_qdot);
    
    // Learning rates for the gradient descent
    nh.getParam("k_mu", k_mu);
    nh.getParam("k_a", k_a);

    // Proportional tuning parameter for the ReAIC
    nh.getParam("Kp", Kp);

    for( int i = 0; i < SigmaP_yq0.rows(); i = i + 1 ) {
      SigmaP_yq0(i,i) = 1/var_q;
      SigmaP_yq1(i,i) = 1/var_qdot;
      SigmaP_mu(i,i) = 1/var_mu;
      SigmaP_muprime(i,i) = 1/var_muprime;

      k_a_adapt(i, i) = k_a;
      k_mu_adapt(i, i) = k_mu;
      k_p_adapt(i,i) = Kp;

      mu(i) = 0.0;
      mu_p(i) = 0.0;

      mu_d(i) = 0.0;
      mu_p_d(i) = 0.0;
    }

    // Initialize control actions
    u << 0.0, 0.0, 0.0, 0.0, 0.0;

    // Initialize prior beliefs about the second ordet derivatives of the states of the robot
    mu_pp << 0.0, 0.0, 0.0, 0.0, 0.0;

    // Integration step
    h = 0.001;

    // Resize Float64MultiArray messages
    AIC_mu.data.resize(5);
    AIC_mu_p.data.resize(5);
    AIC_mu_pp.data.resize(5);
    mu_des.data.resize(5);
    SPE.data.resize(2);
  }

  void ReAIC::minimiseF(int stop){
    // Compute single sensory prediction errors
    SPEq = (jointPos.transpose()-mu.transpose())*SigmaP_yq0*(jointPos-mu);
    SPEdq = (jointVel.transpose()-mu_p.transpose())*SigmaP_yq1*(jointVel-mu_p);
    SPEmu_p = (mu_p.transpose()+Kp*mu.transpose()-Kp*mu_d.transpose())*SigmaP_mu*(mu_p+Kp*mu-Kp*mu_d);
    SPEmu_pp = (mu_pp.transpose()+Kp*mu_p.transpose())*SigmaP_muprime*(mu_pp+Kp*mu_p);

    // Free-energy as a sum of squared values (i.e. sum the SPE)
    F.data = SPEq + SPEdq + SPEmu_p + SPEmu_pp; 

    // Free-energy minimization using gradient descent and beliefs update
    mu_dot = mu_p - k_mu_adapt*(-SigmaP_yq0*(jointPos-mu)+SigmaP_mu*(mu_p+Kp*mu-Kp*mu_d));
    mu_dot_p = mu_pp - k_mu_adapt*(-SigmaP_yq1*(jointVel-mu_p)+SigmaP_mu*(mu_p+Kp*mu-Kp*mu_d)+SigmaP_muprime*(mu_pp+Kp*mu_p));
    mu_dot_pp = - k_mu_adapt*(SigmaP_muprime*(mu_pp+Kp*mu_p));

    // Beliefs update
    mu = mu + h*mu_dot;             // Belief about the position
    mu_p = mu_p + h*mu_dot_p;       // Belief about motion of mu
    mu_pp = mu_pp + h*mu_dot_pp;    // Belief about motion of mu'

    // Publish beliefs as Float64MultiArray
    for (int i=0;i<5;i++){
       AIC_mu.data[i] = mu(i);
       AIC_mu_p.data[i] = mu_p(i);
       AIC_mu_pp.data[i] = mu_pp(i);
    }
    // Define SPE message
    SPE.data[0] = SPEq;
    SPE.data[1] = SPEdq;

    // Calculate and send control actions
    ReAIC::computeActions(stop);

    // Publish free-energy
    IFE_pub.publish(F);

    // Sensory prediction error publisher
    SPE_pub.publish(SPE);

    // Publish beliefs
    beliefs_mu_pub.publish(AIC_mu);
    beliefs_mu_p_pub.publish(AIC_mu_p);
    beliefs_mu_pp_pub.publish(AIC_mu_pp);
  }

  // Compute control actions through gradient descent of F
  void   ReAIC::computeActions(int stop){

    // learning rate is set to zero when within +-0.01 rad of goal
    ReAIC::adjust_learning_rate();  
    
    if (stop == 0){
      u = u-590.0*h*k_a_adapt*(SigmaP_yq1*(jointVel-mu_p)+SigmaP_yq0*(jointPos-mu)); // 590 = torque -> PWM conversion factor
    } else {
      for( int i = 0; i < u.rows(); i = i + 1 ) {
        u(i) = 0.0;
      }
    }

    // Prevent control action windup
    for( int i = 0; i < u.rows(); i = i + 1 ) {
      if (u(i) > 885.0 ){
        u(i) = 885.0;
      }
      else if (u(i) < -885.0){
        u(i) = -885.0;
      }
    }

    

    //ROS_INFO_STREAM("Sending random velocity command:"
    //  << " u= " << u(0) << " " << u(1) << " " << u(2) << " " << u(3) << " " << u(4));
    
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
    singlePub.publish(wrist_ang_msg);
    //singlePub.publish(wrist_rot_msg);

    a.cmd = {waist_msg.cmd, shoulder_msg.cmd, elbow_msg.cmd, wrist_ang_msg.cmd, wrist_rot_msg.cmd};

    //ROS_INFO_STREAM("Sending random velocity command:"
    //  << " Name= " << a.name
    //  << " Cmd= " << a.cmd[0] << " " << a.cmd[1] << " " << a.cmd[2] << " " << a.cmd[3] << " " << a.cmd[4]);
    
    // Publish the message as a group.
    //groupPub.publish(a);
    
    // Set the toques from u and publish for gazebo setup
    tau1.data = u(0); tau2.data = u(1); tau3.data = u(2); tau4.data = u(3);
    tau5.data = u(4); //tau6.data = u(5); //tau7.data = u(6);
    // Publishing
    tauPub1.publish(tau1); tauPub2.publish(tau2); tauPub3.publish(tau3);
    tauPub4.publish(tau4); tauPub5.publish(tau5); //tauPub6.publish(tau6);
    //tauPub7.publish(tau7);
  }

  // Method to observe if the joint states have been received already,
  // used in the main function
  int ReAIC::dataReady(){
    if(dataReceived==1)
      return 1;
    else
      return 0;
  }

  // function to publish the goal states
  void ReAIC::setGoal(std::vector<double> desiredPos){
    for(int i=0; i<desiredPos.size(); i++){
      mu_d(i) = desiredPos[i];
      mu_p_d(i) = 0.0;
      mu_des.data[i] = mu_d(i);
    }
    mu_desired_pub.publish(mu_des);
  }

  // function to return the sensory prediction error
  std_msgs::Float64MultiArray  ReAIC::getSPE(){
    return(SPE);
  }

  // funtion to set the control action learning rate to zero when within 0.01 rad of the goal
  void ReAIC::adjust_learning_rate() {
    error = jointPos - mu_d;
    
    for (int i = 0; i < 5; i++) {
        if (abs(error(i, 0)) < 0.01 && jointVel(i) == 0.0) {
            k_a_adapt(i, i) = 0.0;
        } else {
            k_a_adapt(i, i) = k_a;
        }
    }
  }
