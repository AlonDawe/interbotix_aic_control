import rosbag
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import rcParams
import tikzplotlib
#matplotlib2tikz.save("mytikz.tex")
from mpl_toolkits.axes_grid1.inset_locator import inset_axes,  mark_inset

# Set font and size to match LaTeX document
rcParams['font.family'] = 'serif'
#rcParams['font.serif'] = ['Computer Modern']
rcParams['font.size'] = 10

def ITAE(data, time, ref):
    error = abs(data - ref)
    dt = time[1] - time[0]
    ITAE = np.sum(error * time * dt)
    return ITAE

def error(data, ref):
    error = data - ref
    #dt = time[1] - time[0]
    #ITAE = np.sum(error * time * dt)
    return error

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

def tikzplotlib_fix_ncols(obj):
    """
    workaround for matplotlib 3.6 renamed legend's _ncol to _ncols, which breaks tikzplotlib
    """
    if hasattr(obj, "_ncols"):
        obj._ncol = obj._ncols
    for child in obj.get_children():
        tikzplotlib_fix_ncols(child)
            
            

# Step Response
bag_path1 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/ReAIC_Kp_3_Ka_4.bag"
#bag_path2 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/ReAIC_Kp_6_Ka_2.bag"
#bag_path3 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/ReAIC_Kp_6_Ka_3.bag"
#bag_path4 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/ReAIC_Kp_6_Ka_4.bag"
#bag_path5 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/ReAIC_Kp_6_Ka_5.bag"
##############NOISE#######################
#bag_path1 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/ReAIC_WRIST_Kp_3_Ka_4_NOISE_var_1.bag"
bag_path2 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/ReAIC_WRIST_Kp_3_Ka_4_NOISE_var_2.bag"
bag_path3 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/ReAIC_WRIST_Kp_3_Ka_4_NOISE_var_3.bag"
bag_path4 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/ReAIC_WRIST_Kp_2_Ka_4_NOISE_var_3_varmu_1_2024-02-29-14-26-53.bag"
bag_path5 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/ReAIC_WRIST_Kp_2_Ka_4_NOISE_var_3_varmu_1.bag"

###################
#bag_path5 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/ReAIC_WRIST_Kp_3_Ka_4_NOISE_var_3_varmu_3.bag"

#bag_path1 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/ReAIC_WAIST_Kp_2_Ka_5p5_2024-02-27-14-29-40.bag"
#bag_path2 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/ReAIC_WAIST_Kp_3_Ka_5p5_2024-02-27-14-30-46.bag"
#bag_path3 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/ReAIC_WAIST_Kp_3_Ka_5p5_2024-02-27-16-34-06.bag"
#bag_path4 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/ReAIC_WAIST_Kp_4_Ka_5p5_2024-02-27-14-33-41.bag"
#bag_path5 = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/ReAIC_WAIST_Kp_3_Ka_5_2024-02-27-16-26-43.bag"


# Open the bag file
bag1 = rosbag.Bag(bag_path1)
bag2 = rosbag.Bag(bag_path2)
bag3 = rosbag.Bag(bag_path3)
bag4 = rosbag.Bag(bag_path4)
bag5 = rosbag.Bag(bag_path5)

controllers = ["Ka = 1", "Ka = 2", "Ka = 3", "Ka = 4", "Ka = 5"]

parameter_tuning = np.zeros((5, 7, 3))

#print(parameter_tuning)

bags = [bag1, bag2, bag3, bag4, bag5]

REF_timestamps = []
PID_timestamps = []
AIC_timestamps = []
ReAIC_timestamps = []
AFC_timestamps = []
Tune_timestamps = []

REF_datavalues = []
PID_datavalues = []
AIC_datavalues = []
ReAIC_datavalues = []
AFC_datavalues = []
Tune_datavalues = []

PID_U_timestamps = []
AIC_U_timestamps = []
ReAIC_U_timestamps = []
AFC_U_timestamps = []
Tune_U_timestamps = []

PID_U_datavalues = []
AIC_U_datavalues = []
ReAIC_U_datavalues = []
AFC_U_datavalues = []
Tune_U_datavalues = []

AIC_mu_timestamps = []
ReAIC_mu_timestamps = []

AIC_mu_datavalues = []
ReAIC_mu_datavalues = []

time = [PID_timestamps, AIC_timestamps, ReAIC_timestamps, AFC_timestamps, Tune_timestamps]
data = [PID_datavalues, AIC_datavalues, ReAIC_datavalues, AFC_datavalues, Tune_datavalues]

control_time = [PID_U_timestamps, AIC_U_timestamps, ReAIC_U_timestamps, AFC_U_timestamps, Tune_U_timestamps]
control_signal = [PID_U_datavalues, AIC_U_datavalues, ReAIC_U_datavalues, AFC_U_datavalues, Tune_U_datavalues]

belief_time = [AIC_mu_timestamps, ReAIC_mu_timestamps]
belief_data = [AIC_mu_datavalues, ReAIC_mu_datavalues]

joint = 4

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
            data_value = msg.data[joint]
            
            if timestamp <= 31.0:
                # Append data to lists
                timestamps_mu_d.append(timestamp)
                data_values_mu_d.append(data_value)
            
            
          
        if topic == '/px150/joint_states':
            # Extract data from the message (replace with your actual message structure)
            #print(t)
            data_value = msg.position[joint]
            
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
        
            
            data_value = msg.data[joint]

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
Tune_step_1_data = []

PID_step_2_data = []
AIC_step_2_data = []
ReAIC_step_2_data = []
AFC_step_2_data = []
Tune_step_2_data = []

PID_step_3_data = []
AIC_step_3_data = []
ReAIC_step_3_data = []
AFC_step_3_data = []
Tune_step_3_data = []

PID_step_4_data = []
AIC_step_4_data = []
ReAIC_step_4_data = []
AFC_step_4_data = []
Tune_step_4_data = []

PID_step_5_data = []
AIC_step_5_data = []
ReAIC_step_5_data = []
AFC_step_5_data = []
Tune_step_5_data = []

PID_step_6_data = []
AIC_step_6_data = []
ReAIC_step_6_data = []
AFC_step_6_data = []
Tune_step_6_data = []

PID_step_7_data = []
AIC_step_7_data = []
ReAIC_step_7_data = []
AFC_step_7_data = []
Tune_step_7_data = []

controller_step_1_data = [PID_step_1_data, AIC_step_1_data, ReAIC_step_1_data, AFC_step_1_data, Tune_step_1_data]
controller_step_2_data = [PID_step_2_data, AIC_step_2_data, ReAIC_step_2_data, AFC_step_2_data, Tune_step_2_data]
controller_step_3_data = [PID_step_3_data, AIC_step_3_data, ReAIC_step_3_data, AFC_step_3_data, Tune_step_3_data]
controller_step_4_data = [PID_step_4_data, AIC_step_4_data, ReAIC_step_4_data, AFC_step_4_data, Tune_step_4_data]
controller_step_5_data = [PID_step_5_data, AIC_step_5_data, ReAIC_step_5_data, AFC_step_5_data, Tune_step_5_data]
controller_step_6_data = [PID_step_6_data, AIC_step_6_data, ReAIC_step_6_data, AFC_step_6_data, Tune_step_6_data]
controller_step_7_data = [PID_step_7_data, AIC_step_7_data, ReAIC_step_7_data, AFC_step_7_data, Tune_step_7_data]

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

PID_error = []
AIC_error = []
ReAIC_error = []
AFC_error = []
Tune_error = []

errors = [PID_error, AIC_error, ReAIC_error, AFC_error, Tune_error]

for i in range(5):
    for index, x in enumerate(time[i]):
        if x < step_1[0]:
            errors[i].append(error(data[i][index], 0.0))
        if x >= step_1[0] and x < step_1[1]:
            #controller_step_1_time[i].append(x - step_1[0])
            controller_step_1_data[i].append(data[i][index])
            errors[i].append(error(data[i][index], step_references[0]))
            
        if x >= step_2[0] and x < step_2[1]:
            #controller_step_2_time[i].append(x - step_2[0])
            controller_step_2_data[i].append(data[i][index])
            errors[i].append(error(data[i][index], step_references[1]))
        
        if x >= step_3[0] and x < step_3[1]:
            #controller_step_2_time[i].append(x - step_2[0])
            controller_step_3_data[i].append(data[i][index])
            errors[i].append(error(data[i][index], step_references[2]))
        
        if x >= step_4[0] and x < step_4[1]:
            #controller_step_2_time[i].append(x - step_2[0])
            controller_step_4_data[i].append(data[i][index])
            errors[i].append(error(data[i][index], step_references[3]))
            
        if x >= step_5[0] and x < step_5[1]:
            #controller_step_2_time[i].append(x - step_2[0])
            controller_step_5_data[i].append(data[i][index])
            errors[i].append(error(data[i][index], step_references[4]))
            
        if x >= step_6[0] and x < step_6[1]:
            #controller_step_2_time[i].append(x - step_2[0])
            controller_step_6_data[i].append(data[i][index])
            errors[i].append(error(data[i][index], step_references[5]))
            
        if x >= step_7[0] and x < step_7[1]:
            #controller_step_2_time[i].append(x - step_2[0])
            controller_step_7_data[i].append(data[i][index])
            errors[i].append(error(data[i][index], step_references[6]))

controllers_ITAE = np.zeros((7, 5)) 
controllers_OS = np.zeros((7, 5)) 
controllers_ST = np.zeros((7, 5))
#controllers_error = np.zeros((7, 5))


            
        
for step, controller_step_x_data in enumerate(controller_step_data):
    
    PID_length = len(controller_step_x_data[0])
    AIC_length = len(controller_step_x_data[1])
    ReAIC_length = len(controller_step_x_data[2])
    AFC_length = len(controller_step_x_data[3])
    Tune_length = len(controller_step_x_data[4])

    lengths = np.array([PID_length, AIC_length, ReAIC_length, AFC_length, Tune_length])
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
    Tune_ITAE = ITAE(np.array(controller_step_x_data[4]), controller_step_time[step], step_references[step])
    
    PID_ST = settl_time(np.array(controller_step_x_data[0]), controller_step_time[step], step_references[step])
    AIC_ST = settl_time(np.array(controller_step_x_data[1]), controller_step_time[step], step_references[step])
    ReAIC_ST = settl_time(np.array(controller_step_x_data[2]), controller_step_time[step], step_references[step])
    AFC_ST = settl_time(np.array(controller_step_x_data[3]), controller_step_time[step], step_references[step])
    Tune_ST = settl_time(np.array(controller_step_x_data[4]), controller_step_time[step], step_references[step])
    
    ##for item in controller_step_x_data[0]:
    #    PID_error.append(error(item, step_references[step]))
    
    #for item in controller_step_x_data[1]:
    #    AIC_error.append(error(item, step_references[step]))
        
    #for item in controller_step_x_data[2]:
    #    ReAIC_error.append(error(item, step_references[step]))
    
    #for item in controller_step_x_data[3]:
    #    AFC_error.append(error(item, step_references[step]))
        
    #for item in controller_step_x_data[4]:
    #    Tune_error.append(error(item, step_references[step]))
    
    
    #PID_error.append(error(np.array(controller_step_x_data[0]), step_references[step])[:])
    #AIC_error.append(error(np.array(controller_step_x_data[1]), step_references[step])[:])
    #ReAIC_error.append(error(np.array(controller_step_x_data[2]), step_references[step])[:])
    #AFC_error.append(error(np.array(controller_step_x_data[3]), step_references[step])[:])
    #Tune_error.append(error(np.array(controller_step_x_data[4]), step_references[step])[:])
    
    
    
    controllers_ITAE[step, 0] = PID_ITAE
    controllers_ITAE[step, 1] = AIC_ITAE
    controllers_ITAE[step, 2] = ReAIC_ITAE
    controllers_ITAE[step, 3] = AFC_ITAE
    controllers_ITAE[step, 4] = Tune_ITAE
    
    controllers_ST[step, 0] = PID_ST
    controllers_ST[step, 1] = AIC_ST
    controllers_ST[step, 2] = ReAIC_ST
    controllers_ST[step, 3] = AFC_ST
    controllers_ST[step, 4] = Tune_ST
    
    #controllers_error[step, 0] = PID_error
    #controllers_error[step, 1] = AIC_error
    #controllers_error[step, 2] = ReAIC_error
    #controllers_error[step, 3] = AFC_error
    #controllers_error[step, 4] = Tune_error
    
    print("STEP {} Performance Analysis: ".format(step+1))
    print("========================================")
    print("Ka = 1 ITAE: ", PID_ITAE)
    print("Ka = 2 ITAE: ", AIC_ITAE)
    print("Ka = 3 ITAE: ", ReAIC_ITAE)
    print("Ka = 4 ITAE: ", AFC_ITAE)
    print("Ka = 5 ITAE: ", Tune_ITAE)
    print("========================================")
    print("Ka = 1 ST: ", PID_ST)
    print("Ka = 2 ST: ", AIC_ST)
    print("Ka = 3 ST: ", ReAIC_ST)
    print("Ka = 4 ST: ", AFC_ST)
    print("Ka = 5 ST: ", Tune_ST)
    print("========================================")
    
    
    parameter_tuning[0, step, 0] = PID_ITAE
    parameter_tuning[1, step, 0] = AIC_ITAE
    parameter_tuning[2, step, 0] = ReAIC_ITAE
    parameter_tuning[3, step, 0] = AFC_ITAE
    parameter_tuning[4, step, 0] = Tune_ITAE
    
    parameter_tuning[0, step, 1] = PID_ST
    parameter_tuning[1, step, 1] = AIC_ST
    parameter_tuning[2, step, 1] = ReAIC_ST
    parameter_tuning[3, step, 1] = AFC_ST
    parameter_tuning[4, step, 1] = Tune_ST
    
    
    
    
    
    if step > 1 and step < 6:
        PID_max = np.min(controller_step_x_data[0])
        AIC_max = np.min(controller_step_x_data[1])
        ReAIC_max = np.min(controller_step_x_data[2])
        AFC_max = np.min(controller_step_x_data[3])
        Tune_max = np.min(controller_step_x_data[4])
        controller_max = [PID_max, AIC_max, ReAIC_max, AFC_max, Tune_max]
        
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
            
            parameter_tuning[controller, step, 2] = OS
            
            
    else:
        PID_max = np.max(controller_step_x_data[0])
        AIC_max = np.max(controller_step_x_data[1])
        ReAIC_max = np.max(controller_step_x_data[2])
        AFC_max = np.max(controller_step_x_data[3])
        Tune_max = np.max(controller_step_x_data[4])
        controller_max = [PID_max, AIC_max, ReAIC_max, AFC_max, Tune_max]
        
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
            parameter_tuning[controller, step, 2] = OS
    print("\n")
    
#np.save('parameter_grid_Kp_6', parameter_tuning) 

column_ITAE_variances = np.std(controllers_ITAE, axis=0) 
column_ST_variances = np.std(controllers_ST, axis=0) 
column_OS_variances = np.std(controllers_OS, axis=0)

column_ITAE_mean = np.mean(controllers_ITAE, axis=0) 
column_ST_mean = np.mean(controllers_ST, axis=0) 
column_OS_mean = np.mean(controllers_OS, axis=0)

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

for i in range(5):
    plt.step(time[i], data[i], '-', label=controllers[i])
        
plt.step(timestamps_mu_d, data_values_mu_d, 'k-', label='Desired Position')
plt.hlines(0.0, 0.0, xmax=30.0, color='k', linestyle='--')
# Add labels and legend if necessary
plt.grid(True)
plt.xlabel('Time (s)')
plt.ylabel('Joint Position (rad)')
plt.legend()
plt.title('Wrist Rotation Step Responses')

print("Data size: ", len(data[0]))
print("PID_error size: ", len(PID_error))
print("time size: ", len(time[0]))

# Create subplots
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# Add a title to the entire figure
#plt.suptitle('Wrist Rotation Step Responses with Noise')

# Plot joint position on the first subplot
ax1.step(time[0], data[0], '-', label='$\mathcal{K}_{p} = 3$; $\sigma_{q} = \sigma_{\dot{q}} = 1$', zorder = 4)
ax1.step(time[1], data[1], '-', label='$\mathcal{K}_{p} = 3$; $\sigma_{q} = \sigma_{\dot{q}} = 2$', zorder = 1)
ax1.step(time[2], data[2], '-', label='$\mathcal{K}_{p} = 3$; $\sigma_{q} = \sigma_{\dot{q}} = 3$', zorder = 2)
ax1.step(time[3], data[3], '-', label='$\mathcal{K}_{p} = 2$; $\sigma_{q} = \sigma_{\dot{q}} = 3$', zorder = 3)
#ax1.step(time[4], data[4], '-', label='Ka = 5 Joint Position (rad)')
ax1.step(REF_timestamps, REF_datavalues, color='k', linestyle='--', label='$\mu_{g}$', zorder = 0)

#ax1.hlines(0.0, 0.0, xmax=30.0, color='k', linestyle='--')
ax1.set_ylabel('Joint Angle (rad)')
ax1.legend(ncol=3, fontsize='small', frameon=False, loc='upper center', bbox_to_anchor =(0.5, 1.35))#fontsize='x-small'
ax1.grid(True)  # Turn on the grid for the first subplot
ax1.set_xlim(0, 30)

# Plot control signal on the second subplot
ax2.step(control_time[0], control_signal[0], '-', zorder = 4) #, label='Ka = 1 Control Signal (v)')
ax2.step(control_time[1], control_signal[1], '-', zorder = 1)#, #label='Ka = 2 Control Signal (v)')
ax2.step(control_time[2], control_signal[2], '-', zorder = 2)#, #label='Ka = 3 Control Signal (v)')
ax2.step(control_time[3], control_signal[3], '-', zorder = 3)#, #label='Ka = 4 Control Signal (v)')
#ax2.step(control_time[4], control_signal[4], '-', label='Ka = 5 Control Signal (v)')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Control Signal (V)')
#ax2.legend(fontsize='x-small')
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

#fig.savefig('/home/alon/Documents/thesis/Tuning Parameter Influences/Kp3_Ka4_step_response_with_noise_new.pdf')
#fig.savefig('/home/alon/Documents/thesis/Tuning Parameter Influences/Step_responses_with_noise_new.pdf')

#ax3.step(time[0], PID_error, '-')
#ax3.step(time[1], AIC_error, '-')
#ax3.step(time[2], ReAIC_error, '-')
#ax3.step(time[3], AFC_error, '-')
#ax3.hlines(0.0, 0.0, xmax=30.0, color='k', linestyle='--')
#ax3.set_ylabel('Joint Error ($rad$)')
#ax3.set_xlabel('Time ($s$)')
#ax3.grid(True)  # Turn on the grid for the first subplot

fig5, ax5 = plt.subplots(1, 1, sharex=True)

# Plot joint position on the first subplot
ax5.step(time[0], PID_error, '-')
ax5.step(time[1], AIC_error, '-')
ax5.step(time[2], ReAIC_error, '-')
ax5.step(time[3], AFC_error, '-')
ax5.hlines(0.0, 0.0, xmax=30.0, color='k', linestyle='--')
ax5.set_ylabel('Joint Error ($rad$)')
ax5.set_xlabel('Time ($s$)')
ax5.set_title('Joint Errors')
#ax3.legend(fontsize='x-small')
ax5.grid(True)  # Turn on the grid for the first subplot
#tikzplotlib_fix_ncols(fig)
#tikzplotlib.save("/home/alon/Documents/thesis/Tuning Parameter Influences/Step_responses_with_noise.tex")
#fig.savefig('/home/alon/Documents/thesis/Tuning Parameter Influences/Step_responses_with_noise_var_only.pdf')


for i in range(5):
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

    # Add a title to the entire figure
    plt.suptitle(controllers[i]+' Wrist Rotation Step Responses')

    # Plot joint position on the first subplot
    ax1.step(time[i], data[i], '-', label=controllers[i]+' Joint Position (rad)')
    if i == 1 or i == 2:
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
    

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# Add a title to the entire figure
#plt.suptitle('Wrist Rotation Step Response with Noise [$\mathcal{K}_{p}$: 3; $\kappa_{a}$: 4]')

# Plot joint position on the first subplot
ax1.step(time[0], data[0], '-', label='$\mathcal{K}_{p} = 3$; $\kappa_{a} = 4$')
ax1.step(REF_timestamps, REF_datavalues, color ='k', linestyle='--', label='$\mu_{g}$', zorder=0)
#ax1.hlines(0.0, 0.0, xmax=30.0, color='k', linestyle='--')
ax1.set_ylabel('Joint Angle (rad)')
ax1.legend(ncol=2, fontsize='small', frameon=False, loc='upper center', bbox_to_anchor =(0.5, 1.2))
ax1.grid(True)  # Turn on the grid for the first subplot
ax1.set_xlim(0, 30)

# Plot control signal on the second subplot
ax2.step(control_time[0], control_signal[0])
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Control Signal (V)')
ax2.grid(True)  # Turn on the grid for the first subplot
ax2.set_xlim(0, 30)
#fig.savefig('/home/alon/Documents/thesis/Tuning Parameter Influences/Kp3_Ka4_step_response_with_noise_new.pdf')
#fig.savefig('/home/alon/Documents/thesis/Tuning Parameter Influences/Kp3_Ka4_step_response_new.pdf')


step_x = [1, 2, 3, 4, 5, 6, 7]

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
# Add a title to the entire figure
plt.suptitle('Controller Performance Analysis')

# Plot joint position on the first subplot
ax1.plot(step_x, controllers_ITAE[:, 0], '-o', label='Ka = 1')
ax1.plot(step_x, controllers_ITAE[:, 1], '-o', label='Ka = 2')
ax1.plot(step_x, controllers_ITAE[:, 2], '-o', label='Ka = 3')
ax1.plot(step_x, controllers_ITAE[:, 3], '-o', label='Ka = 4')
ax1.plot(step_x, controllers_ITAE[:, 4], '-o', label='Ka = 5')
#ax1.plot(REF_timestamps, REF_datavalues, 'k-', label='Desired Position')
#ax1.hlines(0.0, 0.0, xmax=30.0, color='k', linestyle='--')
ax1.set_ylabel('Controller ITAE Score')
ax1.legend()
ax1.grid(True)  # Turn on the grid for the first subplot

# Plot control signal on the second subplot
ax2.plot(step_x, controllers_OS[:, 0], '-o', label='Ka = 1')
ax2.plot(step_x, controllers_OS[:, 1], '-o', label='Ka = 2')
ax2.plot(step_x, controllers_OS[:, 2], '-o', label='Ka = 3')
ax2.plot(step_x, controllers_OS[:, 3], '-o', label='Ka = 4')
ax2.plot(step_x, controllers_OS[:, 4], '-o', label='Ka = 5')
ax2.set_xlabel('Step')
ax2.set_ylabel('Controller Overshoot (%)')
ax2.legend()
ax2.grid(True)  # Turn on the grid for the first subplot

ax3.plot(step_x, controllers_ST[:, 0], '-o', label='Ka = 1')
ax3.plot(step_x, controllers_ST[:, 1], '-o', label='Ka = 2')
ax3.plot(step_x, controllers_ST[:, 2], '-o', label='Ka = 3')
ax3.plot(step_x, controllers_ST[:, 3], '-o', label='Ka = 4')
ax3.plot(step_x, controllers_ST[:, 4], '-o', label='Ka = 5')
ax3.set_xlabel('Step')
ax3.set_ylabel('Controller Settling Time (s)')
ax3.legend()
ax3.grid(True)  # Turn on the grid for the first subplot

fig4, ax4 = plt.subplots(1, 1, sharex=True)

ax4.step(timestamps_mu_d, data_values_mu_d, color='k', linestyle='--', label='$\mu_{g}$')
#ax4.hlines(0.0, 0.0, xmax=30.0, color='k', linestyle='--')
# Add labels and legend if necessary
ax4.grid(True)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Joint Angle (rad)')
ax4.legend()
#ax4.set_title('Wrist Rotation Step Response Goals')
ax4.set_xlim(0, 30)

# Add step numbers slightly shifted up and bolded
for i, (step, ref) in enumerate(zip(steps, step_references), start=1):
    step_center = (step[0] + step[1]) / 2
    ax4.text(step_center, ref + 0.05, f'{i}', color='black',
             horizontalalignment='center', verticalalignment='center', fontweight='bold')

#fig4.savefig('/home/alon/Documents/thesis/Tuning Parameter Influences/wrist_rotation_goal_step_responses_new.pdf')


# Show the plot
plt.show()

# Close the bag file when done
bag1.close()
bag2.close()
bag3.close()
bag4.close()
bag5.close()



