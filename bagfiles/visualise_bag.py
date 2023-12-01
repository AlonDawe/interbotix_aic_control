import rosbag
import matplotlib.pyplot as plt

# Trajectory Response
bag_path1 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist/PID_wrist_traj_test_2023-11-22-11-10-15.bag"
bag_path2 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist/ReAIC_wrist_traj_test_2023-11-22-10-24-20.bag"
bag_path3 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist/AIC_wrist_traj_test_2023-11-22-10-36-37.bag"

# Trajectory Load Response
#bag_path1 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist/PID_wrist_traj_load_test_2023-11-22-14-47-47.bag"
#bag_path2 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist/ReAIC_wrist_traj_load_test_2023-11-22-14-46-53.bag"
#bag_path3 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist/AIC_wrist_traj_load_test_2023-11-22-14-45-49.bag"


# Step Response
#bag_path1 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist/PID_wrist_step_test_2023-11-22-14-08-11.bag"
#bag_path1 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist/PID_wrist_step_test_2023-11-22-15-12-42.bag"
#bag_path2 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist/ReAIC_wrist_step_test_2023-11-22-14-18-47.bag"
#bag_path3 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist/AIC_wrist_step_test_2023-11-22-14-18-00.bag"


# Open the bag file
bag1 = rosbag.Bag(bag_path1)
bag2 = rosbag.Bag(bag_path2)
bag3 = rosbag.Bag(bag_path3)

controllers = ["PID", "ReAIC", "AIC"]

bags = [bag1, bag2, bag3]

for index, bag in enumerate(bags):

    # Initialize lists to store data
    timestamps = []
    data_values = []

    timestamps_u = []
    data_values_u = []

    timestamps_mu_d = []
    data_values_mu_d = []
    
    timestamps_mu = []
    data_values_mu = []

    start_time_pos_read = False
    start_time_pos = 0.0
    
    start_time_mu_d_read = False
    start_time_mu_d = 0.0
    
    # Iterate through the messages in the bag
    for topic, msg, t in bag.read_messages():
        # Add your data processing and plotting code here
        # For example, if you have a specific topic containing sensor data:
        if topic == '/mu_desired':
            if start_time_mu_d_read == False:
                start_time_mu_d = t.to_sec()
                start_time_mu_d_read = True
            
            timestamp =  t.to_sec() - start_time_mu_d  # Convert Time to a float
            data_value = msg.data[3]

            # Append data to lists
            timestamps_mu_d.append(timestamp)
            data_values_mu_d.append(data_value)
            
            
        if start_time_mu_d_read == True:   
            if topic == '/px150/joint_states':
                # Extract data from the message (replace with your actual message structure)
                #print(t)
                
                timestamp = msg.header.stamp.to_sec() - start_time_mu_d
                data_value = msg.position[3]
                
                # Append data to lists
                timestamps.append(timestamp)
                
                data_values.append(data_value)
            
            if topic == '/beliefs_mu':
            
                timestamp =  t.to_sec() - start_time_mu_d  # Convert Time to a float
                data_value = msg.data[3]

                # Append data to lists
                timestamps_mu.append(timestamp)
                data_values_mu.append(data_value)
            
        
            

    #print(timestamps_u[0])       
    # Plot the data using a solid red line
    #print(timestamp)
    plt.plot(timestamps, data_values, '-', label=controllers[index])
    
    if index != 0:
        plt.plot(timestamps_mu, data_values_mu, '-', label=controllers[index]+"_mu")
    
    
    #plt.plot(timestamps_u, data_values_u, 'g-', label='/px150/waist_controller/command')
        
plt.plot(timestamps_mu_d, data_values_mu_d, 'k-', label='Desired Position')
plt.hlines(0.0, 0.0, xmax=30.0, color='k', linestyle='--')
# Add labels and legend if necessary
plt.grid(True)
plt.xlabel('Timestamp [sec]')
plt.ylabel('Radians [rad]')
plt.legend()
plt.title('Wrist Trajectory Load (11g) Response')

# Show the plot
plt.show()

# Close the bag file when done
bag1.close()
bag2.close()
bag3.close()

