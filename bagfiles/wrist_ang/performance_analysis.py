import rosbag
import matplotlib.pyplot as plt
import numpy as np
import csv
from matplotlib import rcParams
from mpl_toolkits.axes_grid1.inset_locator import inset_axes,  mark_inset

# Set font and size to match LaTeX document
#rcParams['font.family'] = 'serif'
#rcParams['font.serif'] = ['Computer Modern']
#rcParams['font.size'] = 10

rcParams['font.family'] = 'serif'
rcParams['font.serif'] = ['cmr10']
rcParams['font.size'] = 14
#rcParams['axes.unicode_minus'] = False
rcParams['axes.formatter.use_mathtext'] = True

def save_to_csv(column_headers, column_data, filename):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(column_headers)
        writer.writerow(column_data)

def add_to_csv(column_data, filename):
    with open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        #writer.writerow(column_headers)
        writer.writerow(column_data)

def ITAE(data, time, ref):
    error = abs(data - ref)
    dt = time[1] - time[0]
    ITAE = np.sum(error * time * dt)
    return ITAE

def settl_time(data, time, ref):
    error = abs(data - ref)
    cnt = 0
    settling_time = 5.0
    #print(len(error))
    #print(len(time))
    for idx, err in enumerate(error):
        if idx != 0:
            if cnt == 3 and err < 0.05:
                settling_time = time[idx]
                return settling_time
            elif cnt < 3 and err < 0.05 and error[idx -1] <= err+0.005 and error[idx -1] >= err-0.005 :
                cnt += 1
            else:
                cnt = 0
    return settling_time

# Step Response
#######bag_path1 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/PID_FINAL_I_2024-01-30-09-50-29.bag"
bag_path1 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/PID_CHANGING_I_FINAL_2024-03-04-11-44-35.bag"
bag_path2 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/AIC_FINAL_2023-12-12-11-03-09.bag"
bag_path3 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/ReAIC_FINAL_2024-01-30-09-44-56.bag"
######bag_path4 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/AFC_FINAL_2024-01-25-10-20-46.bag"
bag_path4 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/AFC_CHANGING_I_FINAL_2024-03-04-15-17-29.bag"

# Loaded Step Response
######bag_path1 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/PID_LOAD_FINAL_I_2024-01-30-09-59-50.bag"
#bag_path1 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/PID_LOAD_CHANGING_I_FINAL_2024-03-04-11-47-43.bag"
#bag_path2 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/AIC_LOAD_FINAL_2023-12-12-11-08-49.bag"
#bag_path3 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/ReAIC_LOAD_FINAL_2024-01-30-10-09-09.bag"
#######bag_path4 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/AFC_LOAD_FINAL_2024-01-25-10-29-30.bag"
#bag_path4 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/AFC_LOAD_CHANGING_I_FINAL_2024-03-04-15-11-36.bag"

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
    
    if index == 1 or index == 2:
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
AFC_step_1_data = []

PID_step_2_data = []
AIC_step_2_data = []
ReAIC_step_2_data = []
AFC_step_2_data = []

PID_step_3_data = []
AIC_step_3_data = []
ReAIC_step_3_data = []
AFC_step_3_data = []

PID_step_4_data = []
AIC_step_4_data = []
ReAIC_step_4_data = []
AFC_step_4_data = []

PID_step_5_data = []
AIC_step_5_data = []
ReAIC_step_5_data = []
AFC_step_5_data = []

PID_step_6_data = []
AIC_step_6_data = []
ReAIC_step_6_data = []
AFC_step_6_data = []

PID_step_7_data = []
AIC_step_7_data = []
ReAIC_step_7_data = []
AFC_step_7_data = []

controller_step_1_data = [PID_step_1_data, AIC_step_1_data, ReAIC_step_1_data, AFC_step_1_data]
controller_step_2_data = [PID_step_2_data, AIC_step_2_data, ReAIC_step_2_data, AFC_step_2_data]
controller_step_3_data = [PID_step_3_data, AIC_step_3_data, ReAIC_step_3_data, AFC_step_3_data]
controller_step_4_data = [PID_step_4_data, AIC_step_4_data, ReAIC_step_4_data, AFC_step_4_data]
controller_step_5_data = [PID_step_5_data, AIC_step_5_data, ReAIC_step_5_data, AFC_step_5_data]
controller_step_6_data = [PID_step_6_data, AIC_step_6_data, ReAIC_step_6_data, AFC_step_6_data]
controller_step_7_data = [PID_step_7_data, AIC_step_7_data, ReAIC_step_7_data, AFC_step_7_data]

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


for i in range(4):
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

controllers_ITAE = np.zeros((7, 4)) 
controllers_OS = np.zeros((7, 4)) 
controllers_ST = np.zeros((7, 4))             
        
for step, controller_step_x_data in enumerate(controller_step_data):
    
    PID_length = len(controller_step_x_data[0])
    AIC_length = len(controller_step_x_data[1])
    ReAIC_length = len(controller_step_x_data[2])
    AFC_length = len(controller_step_x_data[3])

    lengths = np.array([PID_length, AIC_length, ReAIC_length, AFC_length])
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
    AFC_ITAE = ITAE(np.array(controller_step_x_data[3]), controller_step_time[step], step_references[step])
    
    PID_ST = settl_time(np.array(controller_step_x_data[0]), controller_step_time[step], step_references[step])
    AIC_ST = settl_time(np.array(controller_step_x_data[1]), controller_step_time[step], step_references[step])
    ReAIC_ST = settl_time(np.array(controller_step_x_data[2]), controller_step_time[step], step_references[step])
    AFC_ST = settl_time(np.array(controller_step_x_data[3]), controller_step_time[step], step_references[step])
    
    controllers_ITAE[step, 0] = PID_ITAE
    controllers_ITAE[step, 1] = AIC_ITAE
    controllers_ITAE[step, 2] = ReAIC_ITAE
    controllers_ITAE[step, 3] = AFC_ITAE
    
    controllers_ST[step, 0] = PID_ST
    controllers_ST[step, 1] = AIC_ST
    controllers_ST[step, 2] = ReAIC_ST
    controllers_ST[step, 3] = AFC_ST
    
    print("STEP {} Performance Analysis: ".format(step+1))
    print("========================================")
    print("PID ITAE: ", PID_ITAE)
    print("AIC ITAE: ", AIC_ITAE)
    print("ReAIC ITAE: ", ReAIC_ITAE)
    print("AFC ITAE: ", AFC_ITAE)
    print("========================================")
    print("PID ST: ", PID_ST)
    print("AIC ST: ", AIC_ST)
    print("ReAIC ST: ", ReAIC_ST)
    print("AFC ST: ", AFC_ST)
    print("========================================")
    
    if step > 1 and step < 6:
        PID_max = np.min(controller_step_x_data[0])
        AIC_max = np.min(controller_step_x_data[1])
        ReAIC_max = np.min(controller_step_x_data[2])
        AFC_max = np.min(controller_step_x_data[3])
        controller_max = [PID_max, AIC_max, ReAIC_max, AFC_max]
        
        for controller, max in enumerate(controller_max):
            
            stopped = False #check if the joint stopped before going past goal
            stopped_cnt = 0
            prev_joint_position = controller_step_x_data[controller][0]
            original_joint_position = controller_step_x_data[controller][0]
            for joint_position in controller_step_x_data[controller][1:]:
                if joint_position <= step_references[step]:
                    break
                if joint_position > original_joint_position:
                    original_joint_position = joint_position
                if joint_position > controller_step_x_data[controller][0]:
                    stopped_cnt = 0
                if joint_position == prev_joint_position and prev_joint_position != original_joint_position: 
                    stopped_cnt += 1
                else:
                    stopped_cnt = 0
                if joint_position > prev_joint_position and joint_position < original_joint_position:
                    stopped = True
                    #print(step, stopped_cnt)
                    #print(joint_position)
                    break
                
                prev_joint_position = joint_position
                
                if stopped_cnt == 3:
                    stopped = True
                    break
                
            if stopped is False:
                max_error = max - step_references[step]
            else:
                max_error = joint_position - step_references[step]
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
        AFC_max = np.max(controller_step_x_data[3])
        controller_max = [PID_max, AIC_max, ReAIC_max, AFC_max]
        
        for controller, max in enumerate(controller_max):
            
            stopped = False #check if the joint stopped before going past goal
            stopped_cnt = 0
            prev_joint_position = controller_step_x_data[controller][0]
            original_joint_position = controller_step_x_data[controller][0]
            for joint_position in controller_step_x_data[controller][1:]:
                if joint_position >= step_references[step]:
                    break
                if joint_position < original_joint_position:
                    original_joint_position = joint_position
                if joint_position < controller_step_x_data[controller][0]:
                    stopped_cnt = 0
                if joint_position == prev_joint_position and prev_joint_position != original_joint_position:
                    stopped_cnt += 1
                elif joint_position != prev_joint_position:
                    stopped_cnt = 0
                if joint_position < prev_joint_position and joint_position > original_joint_position:
                    stopped = True
                    break
                
                prev_joint_position = joint_position
                
                if stopped_cnt == 3:
                    stopped = True
                    break
            if stopped is False:
                max_error = max - step_references[step]
            else: 
                max_error = joint_position - step_references[step]
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
    
column_headers = ["ITAE", "ST", "OS", "ITAE", "ST", "OS", "ITAE", "ST", "OS", "ITAE", "ST", "OS", "ITAE", "ST", "OS", "ITAE", "ST", "OS", "ITAE", "ST", "OS"]

# Data for each column
#controllers_ITAE = [controllers_ITAE[0, 0], "controllers_ITAE[0, 1]", "controllers_ITAE[0, 2]", "controllers_ITAE[0, 3]"]
#controllers_ST = ["controllers_ST[0, 0]", "controllers_ST[0, 1]", "controllers_ST[0, 2]", "controllers_ST[0, 3]"]
#controllers_OS = ["controllers_OS[0, 0]", "controllers_OS[0, 1]", "controllers_OS[0, 2]", "controllers_OS[0, 3]"]

# Save to CSV file
#save_to_csv(column_headers, [controllers_ITAE[0,0], controllers_ST[0,0], controllers_OS[0,0], 
#                             controllers_ITAE[1,0], controllers_ST[1,0], controllers_OS[1,0],
#                             controllers_ITAE[2,0], controllers_ST[2,0], controllers_OS[2,0],
#                             controllers_ITAE[3,0], controllers_ST[3,0], controllers_OS[3,0],
#                             controllers_ITAE[4,0], controllers_ST[4,0], controllers_OS[4,0],
#                             controllers_ITAE[5,0], controllers_ST[5,0], controllers_OS[5,0],
#                             controllers_ITAE[6,0], controllers_ST[6,0], controllers_OS[6,0]], "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/output.csv")
#
#for i in range(1, 4):
#    add_to_csv([controllers_ITAE[0,i], controllers_ST[0,i], controllers_OS[0,i], 
#                             controllers_ITAE[1,i], controllers_ST[1,i], controllers_OS[1,i],
#                             controllers_ITAE[2,i], controllers_ST[2,i], controllers_OS[2,i],
#                             controllers_ITAE[3,i], controllers_ST[3,i], controllers_OS[3,i],
#                             controllers_ITAE[4,i], controllers_ST[4,i], controllers_OS[4,i],
#                             controllers_ITAE[5,i], controllers_ST[5,i], controllers_OS[5,i],
#                             controllers_ITAE[6,i], controllers_ST[6,i], controllers_OS[6,i]], "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/output.csv")
   
   
    

column_ITAE_variances = np.std(controllers_ITAE, axis=0) 
column_ST_variances = np.std(controllers_ST, axis=0)
column_OS_variances = np.std(controllers_OS, axis=0)

column_ITAE_mean = np.mean(controllers_ITAE, axis=0) 
column_ST_mean = np.mean(controllers_ST, axis=0) 
column_OS_mean = np.mean(controllers_OS, axis=0)

#save_to_csv(["mean", "std", "mean", "std", "mean", "std"], [column_ITAE_mean[0], column_ITAE_variances[0], 
#                                                            column_ST_mean[0], column_ST_variances[0], 
#                                                            column_OS_mean[0], column_OS_variances[0]], "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/mean_std.csv")
#
#for i in range(1, 4):
#    add_to_csv([column_ITAE_mean[i], column_ITAE_variances[i], 
#                column_ST_mean[i], column_ST_variances[i], 
#                column_OS_mean[i], column_OS_variances[i]], "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/wrist_ang/mean_std.csv")


print("\n")
print("ITAE Variances: ", column_ITAE_variances)
print("ST Variances: ", column_ST_variances)
print("OS Variances: ", column_OS_variances)

print("\n")

print("ITAE Mean: ", column_ITAE_mean)
print("ST Mean: ", column_ST_mean)
print("OS Mean: ", column_OS_mean)
 

REF_timestamps = timestamps_mu_d
REF_datavalues =  data_values_mu_d

#print(REF_timestamps)

for i in range(4):
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
#plt.suptitle('Wrist Angle Step Responses with 35$g$ Payload')

# Plot joint position on the first subplot
ax1.step(time[0], data[0], '-', label='PID')
ax1.step(time[1], data[1], '-', label='AIC')
ax1.step(time[2], data[2], '-', label='ReAIC')
ax1.step(time[3], data[3], '-', label='AFC')
ax1.step(REF_timestamps, REF_datavalues, color='k', linestyle='--', label='$\mu_{g}$' , zorder=0)
#ax1.hlines(0.0, 0.0, xmax=30.0, color='k', linestyle='--')
ax1.set_ylabel('Joint Angle (rad)')
ax1.legend(ncol=5, fontsize='small', frameon=False, loc='upper center', bbox_to_anchor =(0.5, 1.28))
ax1.grid(True)  # Turn on the grid for the first subplot
ax1.set_xlim(0, 30)

# Plot control signal on the second subplot
ax2.step(control_time[0], control_signal[0], '-')#, label='PID Control Signal (v)')
ax2.step(control_time[1], control_signal[1], '-')#, label='AIC Control Signal (v)')
ax2.step(control_time[2], control_signal[2], '-')#, label='ReAIC Control Signal (v)')
ax2.step(control_time[3], control_signal[3], '-')#, label='AFC Control Signal (v)')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Control Signal (V)')
#ax2.legend()
ax2.grid(True)  # Turn on the grid for the first subplot
ax2.set_xlim(0, 30)

# Create inset axes for the zoomed-in view
axins = inset_axes(ax1, width="45%", height="45%", loc='upper right')
axins.step(time[0], data[0], '-')
axins.step(time[1], data[1], '-')
axins.step(time[2], data[2], '-')
axins.step(time[3], data[3], '-')
axins.step(REF_timestamps, REF_datavalues, 'k', linestyle='--', zorder = 0)
axins.set_xlim(13, 18)
axins.set_ylim(-0.35, 0.35)
plt.xticks(visible=False)
plt.yticks(visible=False)

ax1.indicate_inset_zoom(axins, edgecolor="black")
# Mark the inset area and draw connecting lines
mark_inset(ax1, axins, loc1=2, loc2=4, fc="none", ec="0.7")

#fig.savefig('/home/alon/Documents/thesis/Latex Experiment Graphs/wrist_angle_test_new.pdf')
#fig.savefig('/home/alon/Documents/thesis/Latex Experiment Graphs/wrist_angle_load_test_new.pdf')

for i in range(4):
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

    # Add a title to the entire figure
    plt.suptitle(controllers[i]+' Waist Angle Step Responses')

    # Plot joint position on the first subplot
    ax1.plot(time[i], data[i], '-', label=controllers[i]+' Joint Position (rad)')
    if i == 1 or i == 2:
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

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
# Add a title to the entire figure
plt.suptitle('Controller Performance Analysis')

# Plot joint position on the first subplot
ax1.plot(step_x, controllers_ITAE[:, 0], '-o', label='PID')
ax1.plot(step_x, controllers_ITAE[:, 1], '-o', label='AIC')
ax1.plot(step_x, controllers_ITAE[:, 2], '-o', label='ReAIC')
ax1.plot(step_x, controllers_ITAE[:, 3], '-o', label='AFC')
#ax1.plot(REF_timestamps, REF_datavalues, 'k-', label='Desired Position')
#ax1.hlines(0.0, 0.0, xmax=30.0, color='k', linestyle='--')
ax1.set_ylabel('Controller ITAE Score')
ax1.legend()
ax1.grid(True)  # Turn on the grid for the first subplot

# Plot control signal on the second subplot
ax2.plot(step_x, controllers_OS[:, 0], '-o', label='PID')
ax2.plot(step_x, controllers_OS[:, 1], '-o', label='AIC')
ax2.plot(step_x, controllers_OS[:, 2], '-o', label='ReAIC')
ax2.plot(step_x, controllers_OS[:, 3], '-o', label='AFC')
ax2.set_xlabel('Step')
ax2.set_ylabel('Controller Overshoot (%)')
ax2.legend()
ax2.grid(True)  # Turn on the grid for the first subplot

ax3.plot(step_x, controllers_ST[:, 0], '-o', label='PID')
ax3.plot(step_x, controllers_ST[:, 1], '-o', label='AIC')
ax3.plot(step_x, controllers_ST[:, 2], '-o', label='ReAIC')
ax3.plot(step_x, controllers_ST[:, 3], '-o', label='AFC')
ax3.set_xlabel('Step')
ax3.set_ylabel('Controller Settling Time (s)')
ax3.legend()
ax3.grid(True)  # Turn on the grid for the first subplot

# Show the plot
plt.show()

# Close the bag file when done
bag1.close()
bag2.close()
bag3.close()
bag4.close()



