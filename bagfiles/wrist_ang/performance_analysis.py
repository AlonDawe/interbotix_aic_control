import rosbag
import matplotlib.pyplot as plt
import numpy as np

def ITAE(data, time, ref):
    error = abs(data - ref)
    dt = time[1] - time[0]
    ITAE = np.sum(error * time * dt)
    return ITAE

# Step Response
#bag_path1 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/PID_FINAL_2023-12-12-11-05-16.bag"
#bag_path2 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/AIC_FINAL_2023-12-12-11-03-09.bag"
#bag_path3 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/ReAIC_FINAL_2023-12-12-11-04-11.bag"

# Loaded Step Response
bag_path1 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/PID_LOAD_FINAL_2023-12-12-11-29-04.bag"
bag_path2 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/AIC_LOAD_FINAL_2023-12-12-11-08-49.bag"
bag_path3 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/ReAIC_LOAD_FINAL_2023-12-12-11-28-08.bag"


# Open the bag file
bag1 = rosbag.Bag(bag_path1)
bag2 = rosbag.Bag(bag_path2)
bag3 = rosbag.Bag(bag_path3)

controllers = ["PID", "AIC", "ReAIC"]

bags = [bag1, bag2, bag3]

REF_timestamps = []
PID_timestamps = []
AIC_timestamps = []
ReAIC_timestamps = []

REF_datavalues = []
PID_datavalues = []
AIC_datavalues = []
ReAIC_datavalues = []

PID_U_timestamps = []
AIC_U_timestamps = []
ReAIC_U_timestamps = []

PID_U_datavalues = []
AIC_U_datavalues = []
ReAIC_U_datavalues = []

AIC_mu_timestamps = []
ReAIC_mu_timestamps = []

AIC_mu_datavalues = []
ReAIC_mu_datavalues = []

time = [PID_timestamps, AIC_timestamps, ReAIC_timestamps]
data = [PID_datavalues, AIC_datavalues, ReAIC_datavalues]

control_time = [PID_U_timestamps, AIC_U_timestamps, ReAIC_U_timestamps]
control_signal = [PID_U_datavalues, AIC_U_datavalues, ReAIC_U_datavalues]

belief_time = [AIC_mu_timestamps, ReAIC_mu_timestamps]
belief_data = [AIC_mu_datavalues, ReAIC_mu_datavalues]


for index, bag in enumerate(bags):

    # Initialize lists to store data
    timestamps = [0.0]
    data_values = [0.0]

    timestamps_u = [0.0]
    data_values_u = [0.0]

    timestamps_mu_d = []
    data_values_mu_d = []
    
    timestamps_mu = [0]
    data_values_mu = [0]

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
            
            if timestamp <= 31.0:
                # Append data to lists
                timestamps_mu_d.append(timestamp)
                data_values_mu_d.append(data_value)
            
            
          
        if topic == '/px150/joint_states':
            # Extract data from the message (replace with your actual message structure)
            #print(t)
            data_value = msg.position[3]
            
            if start_time_mu_d_read == True: 
                # Append data to lists
                timestamp = t.to_sec() - start_time_mu_d
                if timestamp <= 31.0:
                    timestamps.append(timestamp)
                    data_values.append(data_value)
            else:
                timestamps[0] = 0.0
                data_values[0] = data_value
        
        if topic == '/px150/commands/joint_single':
            data_value = (msg.cmd * 0.113)/100 * 12

            if start_time_mu_d_read == True:
                # Append data to lists
                timestamp = t.to_sec() - start_time_mu_d # Convert Time to a float
                if timestamp <= 31.0:
                    timestamps_u.append(timestamp)
                    data_values_u.append(data_value)
            else:
                timestamps_u[0] = 0.0
                data_values_u[0] = data_value
        
        if topic == '/beliefs_mu':
        
            
            data_value = msg.data[3]

            if start_time_mu_d_read == True:
                # Append data to lists
                timestamp =  t.to_sec() - start_time_mu_d  # Convert Time to a float
                if timestamp <= 31.0:
                    timestamps_mu.append(timestamp)
                    data_values_mu.append(data_value)
            else:
                timestamps_mu[0] = 0.0
                data_values_mu[0] = data_value
            
        
            

    #print(timestamps_u[0])       
    # Plot the data using a solid red line
    #print(timestamp)
    #plt.plot(timestamps, data_values, '-', label=controllers[index])
    
    time[index] = timestamps
    data[index] = data_values
    control_time[index] = timestamps_u
    control_signal[index] = data_values_u
    
    if index != 0:
        belief_time[index-1] = timestamps_mu
        belief_data[index-1] = data_values_mu
    #if index != 0:
    #    plt.plot(timestamps_mu, data_values_mu, '-', label=controllers[index]+"_mu")
    
    
    #plt.plot(timestamps_u, data_values_u, 'g-', label='/px150/waist_controller/command')
    
step_1 = [3.0, 8.0]
step_2 = [8.0, 13.0]
step_3 = [13.0, 18.0]
step_4 = [18.0, 21.0]
step_5 = [21.0, 24.0]
step_6 = [24.0, 27.0]
step_7 = [27.0, 31.0]
steps = [step_1, step_2, step_3, step_4, step_5, step_6, step_7]

step_references = [0.8, 1.6, 0.0, -0.2, -0.4, -0.6, 0.0]

PID_step_1_data = []
AIC_step_1_data = []
ReAIC_step_1_data = []

PID_step_2_data = []
AIC_step_2_data = []
ReAIC_step_2_data = []

PID_step_3_data = []
AIC_step_3_data = []
ReAIC_step_3_data = []

PID_step_4_data = []
AIC_step_4_data = []
ReAIC_step_4_data = []

PID_step_5_data = []
AIC_step_5_data = []
ReAIC_step_5_data = []

PID_step_6_data = []
AIC_step_6_data = []
ReAIC_step_6_data = []

PID_step_7_data = []
AIC_step_7_data = []
ReAIC_step_7_data = []

controller_step_1_data = [PID_step_1_data, AIC_step_1_data, ReAIC_step_1_data]
controller_step_2_data = [PID_step_2_data, AIC_step_2_data, ReAIC_step_2_data]
controller_step_3_data = [PID_step_3_data, AIC_step_3_data, ReAIC_step_3_data]
controller_step_4_data = [PID_step_4_data, AIC_step_4_data, ReAIC_step_4_data]
controller_step_5_data = [PID_step_5_data, AIC_step_5_data, ReAIC_step_5_data]
controller_step_6_data = [PID_step_6_data, AIC_step_6_data, ReAIC_step_6_data]
controller_step_7_data = [PID_step_7_data, AIC_step_7_data, ReAIC_step_7_data]

controller_step_data = [controller_step_1_data, controller_step_2_data, controller_step_3_data, 
                        controller_step_4_data, controller_step_5_data, controller_step_6_data,
                        controller_step_7_data]

controller_step_1_time = []
controller_step_2_time = []
controller_step_3_time = []
controller_step_4_time = []
controller_step_5_time = []
controller_step_6_time = []
controller_step_7_time = []

controller_step_time = [controller_step_1_time, controller_step_2_time, controller_step_3_time, 
                        controller_step_4_time, controller_step_5_time, controller_step_6_time,
                        controller_step_7_time]


for i in range(3):
    for index, x in enumerate(time[i]):
        if x >= step_1[0] and x < step_1[1]:
            #controller_step_1_time[i].append(x - step_1[0])
            controller_step_1_data[i].append(data[i][index])
            
        if x >= step_2[0] and x < step_2[1]:
            #controller_step_2_time[i].append(x - step_2[0])
            controller_step_2_data[i].append(data[i][index])
        
        if x >= step_3[0] and x < step_3[1]:
            #controller_step_2_time[i].append(x - step_2[0])
            controller_step_3_data[i].append(data[i][index])
        
        if x >= step_4[0] and x < step_4[1]:
            #controller_step_2_time[i].append(x - step_2[0])
            controller_step_4_data[i].append(data[i][index])
            
        if x >= step_5[0] and x < step_5[1]:
            #controller_step_2_time[i].append(x - step_2[0])
            controller_step_5_data[i].append(data[i][index])
            
        if x >= step_6[0] and x < step_6[1]:
            #controller_step_2_time[i].append(x - step_2[0])
            controller_step_6_data[i].append(data[i][index])
            
        if x >= step_7[0] and x < step_7[1]:
            #controller_step_2_time[i].append(x - step_2[0])
            controller_step_7_data[i].append(data[i][index])

controllers_ITAE = np.zeros((7, 3)) 
controllers_OS = np.zeros((7, 3))             
        
for step, controller_step_x_data in enumerate(controller_step_data):
    
    PID_length = len(controller_step_x_data[0])
    AIC_length = len(controller_step_x_data[1])
    ReAIC_length = len(controller_step_x_data[2])

    lengths = np.array([PID_length, AIC_length, ReAIC_length])
    #print(lengths)
    max_length = np.max(lengths)
    
    for index, length in enumerate(lengths):
        if length < max_length:
            for i in range(max_length - length):
                controller_step_x_data[index].append(controller_step_x_data[index][-1])
            
    controller_step_time[step] = np.linspace(0.0, steps[step][1] - steps[step][0], max_length, endpoint=False)
    
    #print(controller_step_time[step])
    
    PID_ITAE = ITAE(np.array(controller_step_x_data[0]), controller_step_time[step], step_references[step])
    AIC_ITAE = ITAE(np.array(controller_step_x_data[1]), controller_step_time[step], step_references[step])
    ReAIC_ITAE = ITAE(np.array(controller_step_x_data[2]), controller_step_time[step], step_references[step])
    
    controllers_ITAE[step, 0] = PID_ITAE
    controllers_ITAE[step, 1] = AIC_ITAE
    controllers_ITAE[step, 2] = ReAIC_ITAE
    
    print("STEP {} Performance Analysis: ".format(step+1))
    print("========================================")
    print("PID ITAE: ", PID_ITAE)
    print("AIC ITAE: ", AIC_ITAE)
    print("ReAIC ITAE: ", ReAIC_ITAE)
    
    print("========================================")
    if step > 1 and step < 6:
        PID_max = np.min(controller_step_x_data[0])
        AIC_max = np.min(controller_step_x_data[1])
        ReAIC_max = np.min(controller_step_x_data[2])
        controller_max = [PID_max, AIC_max, ReAIC_max]
        
        for controller, max in enumerate(controller_max):
            max_error = max - step_references[step]
            if max_error > 0.0:
                max_error = 0.0
                OS = 0.0
            
            elif step_references[step] == 0.0:
                OS = abs(max_error)*100  
            else:
                OS = max_error/step_references[step]*100    
            print("{} OS%: {}%".format(controllers[controller], OS))
            
            controllers_OS[step, controller] = OS
            
            
    else:
        PID_max = np.max(controller_step_x_data[0])
        AIC_max = np.max(controller_step_x_data[1])
        ReAIC_max = np.max(controller_step_x_data[2])
        controller_max = [PID_max, AIC_max, ReAIC_max]
        
        for controller, max in enumerate(controller_max):
            max_error = max - step_references[step]
            if max_error < 0.0:
                max_error = 0.0
                OS = 0.0
            elif step_references[step] == 0.0:
                OS = abs(max_error)*100  
            else:
                OS = max_error/step_references[step]*100    
            print("{} OS: {}".format(controllers[controller], OS))
            
            controllers_OS[step, controller] = OS
    print("\n")
    

column_ITAE_variances = np.var(controllers_ITAE, axis=0) 
column_OS_variances = np.var(controllers_OS, axis=0)
print("\n")
print("ITAE Variances: ", column_ITAE_variances)
print("OS Variances: ", column_OS_variances)
 

REF_timestamps = timestamps_mu_d
REF_datavalues =  data_values_mu_d

#print(REF_timestamps)

for i in range(3):
    plt.plot(time[i], data[i], '-', label=controllers[i])
        
plt.plot(timestamps_mu_d, data_values_mu_d, 'k-', label='Desired Position')
plt.hlines(0.0, 0.0, xmax=30.0, color='k', linestyle='--')
# Add labels and legend if necessary
plt.grid(True)
plt.xlabel('Time (s)')
plt.ylabel('Joint Position (rad)')
plt.legend()
plt.title('Waist Angle Step Responses')




# Create subplots
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# Add a title to the entire figure
plt.suptitle('Controller Waist Angle Step Responses')

# Plot joint position on the first subplot
ax1.plot(time[0], data[0], '-', label='PID Joint Position (rad)')
ax1.plot(time[1], data[1], '-', label='AIC Joint Position (rad)')
ax1.plot(time[2], data[2], '-', label='ReAIC Joint Position (rad)')
ax1.plot(REF_timestamps, REF_datavalues, 'k-', label='Desired Position')
ax1.hlines(0.0, 0.0, xmax=30.0, color='k', linestyle='--')
ax1.set_ylabel('Joint Position (rad)')
ax1.legend()
ax1.grid(True)  # Turn on the grid for the first subplot

# Plot control signal on the second subplot
ax2.plot(control_time[0], control_signal[0], '-', label='PID Control Signal (v)')
ax2.plot(control_time[1], control_signal[1], '-', label='AIC Control Signal (v)')
ax2.plot(control_time[2], control_signal[2], '-', label='ReAIC Control Signal (v)')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Control Signal (v)')
ax2.legend()
ax2.grid(True)  # Turn on the grid for the first subplot


for i in range(3):
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

    # Add a title to the entire figure
    plt.suptitle(controllers[i]+' Waist Angle Step Responses')

    # Plot joint position on the first subplot
    ax1.plot(time[i], data[i], '-', label=controllers[i]+' Joint Position (rad)')
    if i != 0:
        ax1.plot(belief_time[i-1], belief_data[i-1], '-', label=controllers[i]+' Joint Belief (rad)')
    ax1.plot(REF_timestamps, REF_datavalues, 'k-', label='Desired Position')
    ax1.hlines(0.0, 0.0, xmax=30.0, color='k', linestyle='--')
    ax1.set_ylabel('Joint Position (rad)')
    ax1.legend()
    ax1.grid(True)  # Turn on the grid for the first subplot

    # Plot control signal on the second subplot
    ax2.plot(control_time[i], control_signal[i], '-', label=controllers[i]+' Control Signal (v)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Control Signal (v)')
    ax2.legend()
    ax2.grid(True)  # Turn on the grid for the first subplot


step_x = [1, 2, 3, 4, 5, 6, 7]

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
# Add a title to the entire figure
plt.suptitle('Controller Performance Analysis')

# Plot joint position on the first subplot
ax1.plot(step_x, controllers_ITAE[:, 0], '-o', label='PID')
ax1.plot(step_x, controllers_ITAE[:, 1], '-o', label='AIC')
ax1.plot(step_x, controllers_ITAE[:, 2], '-o', label='ReAIC')
#ax1.plot(REF_timestamps, REF_datavalues, 'k-', label='Desired Position')
#ax1.hlines(0.0, 0.0, xmax=30.0, color='k', linestyle='--')
ax1.set_ylabel('Controller ITAE Score')
ax1.legend()
ax1.grid(True)  # Turn on the grid for the first subplot

# Plot control signal on the second subplot
ax2.plot(step_x, controllers_OS[:, 0], '-o', label='PID')
ax2.plot(step_x, controllers_OS[:, 1], '-o', label='AIC')
ax2.plot(step_x, controllers_OS[:, 2], '-o', label='ReAIC')
ax2.set_xlabel('Step')
ax2.set_ylabel('Controller Overshoot (%)')
ax2.legend()
ax2.grid(True)  # Turn on the grid for the first subplot

# Show the plot
plt.show()

# Close the bag file when done
bag1.close()
bag2.close()
bag3.close()



