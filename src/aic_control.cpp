// This program publishes randomly-generated velocity
// messages for turtlesim.
#include <ros/ros.h>
#include <stdlib.h> // For rand() and RAND_MAX
#include <interbotix_xs_msgs/JointGroupCommand.h>
#include <sensor_msgs/JointState.h>

void jointStateReceived(const sensor_msgs::JointState::ConstPtr& msg){
    ROS_INFO_STREAM("Receiving JointStates:"
        << "position=(" << msg->position[0] << ", " << msg->position[1] << ", " << msg->position[2] << ", " << msg->position[3] << ", " << msg->position[4] << ")"
        << "velocity=(" << msg->velocity[0] << ", " << msg->velocity[1] << ", " << msg->velocity[2] << ", " << msg->velocity[3] << ", " << msg->velocity[4] <<")");
}

int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "aic_control_node");
  ros::NodeHandle nh;

  // Create a publisher object.
  ros::Publisher pub = nh.advertise<interbotix_xs_msgs::JointGroupCommand>("px150/commands/joint_group", 20);
  // Create a subscriber object .
  ros::Subscriber sub = nh.subscribe("/px150/joint_states", 1, &jointStateReceived) ;

  // Seed the random number generator.
  srand(time(0));
  interbotix_xs_msgs::JointGroupCommand msg;
  // Loop at 2Hz until the node is shut down.
  ros::Rate rate(1000);
  while(ros::ok()) {
    ros::spinOnce();
    // Create and fill in the message.  The other four
    // fields, which are ignored by turtlesim, default to 0.
    
    msg.name = "arm";
    msg.cmd.push_back(50);
    msg.cmd.push_back(0);
    msg.cmd.push_back(0);
    msg.cmd.push_back(0);
    msg.cmd.push_back(0);
    msg.cmd.push_back(0);
    
    

    // Publish the message.
    pub.publish(msg);
    // Send a message to rosout with the details.
    ROS_INFO_STREAM("Sending random velocity command:"
      << " Name= " << msg.name
      << " Cmd= " << msg.cmd[0] << " " << msg.cmd[1] << " " << msg.cmd[2] << " " << msg.cmd[3] << " " << msg.cmd[4] << " " << msg.cmd[5]);

    // Wait until it's time for another iteration.
    
    rate.sleep();
    
  }
}