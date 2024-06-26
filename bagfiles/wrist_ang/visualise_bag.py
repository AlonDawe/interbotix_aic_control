import rosbag
import matplotlib.pyplot as plt

# Step Response

#bag_path1 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/PID_FINAL_I_2024-01-30-09-50-29.bag"
#bag_path2 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/AIC_FINAL_2023-12-12-11-03-09.bag"
#bag_path3 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/ReAIC_FINAL_2024-01-30-09-44-56.bag"
#bag_path4 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/AFC_FINAL_2024-01-25-10-20-46.bag"

# Loaded Step Response

bag_path1 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/PID_LOAD_FINAL_I_2024-01-30-09-59-50.bag"
bag_path2 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/AIC_LOAD_FINAL_2023-12-12-11-08-49.bag"
bag_path3 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/ReAIC_LOAD_FINAL_2024-01-30-10-09-09.bag"
bag_path4 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/AFC_LOAD_FINAL_2024-01-25-10-29-30.bag"

# Open the bag file
bag1 = rosbag.Bag(bag_path1)
bag2 = rosbag.Bag(bag_path2)
bag3 = rosbag.Bag(bag_path3)
bag4 = rosbag.Bag(bag_path4)

controllers = ["PID", "AIC", "ReAIC", "AFC"]

bags = [bag1, bag2, bag3, bag4]

REF_timestamps = []
PID_timestamps = []
AIC_timestamps = []
ReAIC_timestamps = []
AFC_timestamps = []

REF_datavalues = []
PID_datavalues = []
AIC_datavalues = []
ReAIC_datavalues = []
AFC_datavalues = []

PID_U_timestamps = []
AIC_U_timestamps = []
ReAIC_U_timestamps = []
AFC_U_timestamps = []

PID_U_datavalues = []
AIC_U_datavalues = []
ReAIC_U_datavalues = []
AFC_U_datavalues = []

AIC_mu_timestamps = []
ReAIC_mu_timestamps = []

AIC_mu_datavalues = []
ReAIC_mu_datavalues = []

time = [PID_timestamps, AIC_timestamps, ReAIC_timestamps, AFC_timestamps]
data = [PID_datavalues, AIC_datavalues, ReAIC_datavalues, AFC_datavalues]

control_time = [PID_U_timestamps, AIC_U_timestamps, ReAIC_U_timestamps, AFC_U_timestamps]
control_signal = [PID_U_datavalues, AIC_U_datavalues, ReAIC_U_datavalues, AFC_U_datavalues]

belief_time = [AIC_mu_timestamps, ReAIC_mu_timestamps]
belief_data = [AIC_mu_datavalues, ReAIC_mu_datavalues]


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
            
            if topic == '/px150/commands/joint_single':
                timestamp = t.to_sec() - start_time_mu_d # Convert Time to a float
                data_value = (msg.cmd * 0.113)/100 * 12

                # Append data to lists
                timestamps_u.append(timestamp)
                data_values_u.append(data_value)
            
            if topic == '/beliefs_mu':
            
                timestamp =  t.to_sec() - start_time_mu_d  # Convert Time to a float
                data_value = msg.data[3]

                # Append data to lists
                timestamps_mu.append(timestamp)
                data_values_mu.append(data_value)
            
        
            

    #print(timestamps_u[0])       
    # Plot the data using a solid red line
    #print(timestamp)
    #plt.plot(timestamps, data_values, '-', label=controllers[index])
    
    time[index] = timestamps
    data[index] = data_values
    control_time[index] = timestamps_u
    control_signal[index] = data_values_u
    
    if index == 1 or index == 2:
        belief_time[index-1] = timestamps_mu
        belief_data[index-1] = data_values_mu
    #if index != 0:
    #    plt.plot(timestamps_mu, data_values_mu, '-', label=controllers[index]+"_mu")
    
    
    #plt.plot(timestamps_u, data_values_u, 'g-', label='/px150/waist_controller/command')
  

REF_timestamps = timestamps_mu_d
REF_datavalues =  data_values_mu_d

for i in range(4):
    plt.step(time[i], data[i], '-', label=controllers[i])
        
plt.step(timestamps_mu_d, data_values_mu_d, 'k-', label='Desired Position')
plt.hlines(0.0, 0.0, xmax=30.0, color='k', linestyle='--')
# Add labels and legend if necessary
plt.grid(True)
plt.xlabel('Time (s)')
plt.ylabel('Joint Position (rad)')
plt.legend()
plt.title('Wrist Angle Loaded Step Responses')




# Create subplots
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# Add a title to the entire figure
#plt.suptitle('Controller Wrist Angle')

# Plot joint position on the first subplot
ax1.step(time[0], data[0], '-', label='PID Joint Position (rad)')
ax1.step(time[1], data[1], '-', label='AIC Joint Position (rad)')
ax1.step(time[2], data[2], '-', label='ReAIC Joint Position (rad)')
ax1.step(time[3], data[3], '-', label='AFC Joint Position (rad)')
ax1.step(REF_timestamps, REF_datavalues, 'k-', label='Desired Position')
ax1.hlines(0.0, 0.0, xmax=30.0, color='k', linestyle='--')
ax1.set_ylabel('Joint Position (rad)')
ax1.legend()
ax1.grid(True)  # Turn on the grid for the first subplot

# Plot control signal on the second subplot
ax2.step(control_time[0], control_signal[0], '-', label='PID Control Signal (v)')
ax2.step(control_time[1], control_signal[1], '-', label='AIC Control Signal (v)')
ax2.step(control_time[2], control_signal[2], '-', label='ReAIC Control Signal (v)')
ax2.step(control_time[3], control_signal[3], '-', label='AFC Control Signal (v)')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Control Signal (v)')
ax2.legend()
ax2.grid(True)  # Turn on the grid for the first subplot


for i in range(4):
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

    # Add a title to the entire figure
    plt.suptitle(controllers[i]+' Wrist Angle Step Responses')

    # Plot joint position on the first subplot
    ax1.step(time[i], data[i], '-', label=controllers[i]+' Joint Position (rad)')
    if i == 1 or i ==2:
        ax1.step(belief_time[i-1], belief_data[i-1], '-', label=controllers[i]+' Joint Belief (rad)')
    ax1.step(REF_timestamps, REF_datavalues, 'k-', label='Desired Position')
    ax1.hlines(0.0, 0.0, xmax=30.0, color='k', linestyle='--')
    ax1.set_ylabel('Joint Position (rad)')
    ax1.legend()
    ax1.grid(True)  # Turn on the grid for the first subplot

    # Plot control signal on the second subplot
    ax2.step(control_time[i], control_signal[i], '-', label=controllers[i]+' Control Signal (v)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Control Signal (v)')
    ax2.legend()
    ax2.grid(True)  # Turn on the grid for the first subplot


# Show the plot
plt.show()

# Close the bag file when done
bag1.close()
bag2.close()
bag3.close()
bag4.close()

