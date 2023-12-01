import rosbag
import matplotlib.pyplot as plt

# Specify the path to your ROS bag file
#bag_path = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/_2023-11-21-14-13-48.bag"
bag_path = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/waist/WAIST_STEP_2023-11-29-09-24-21.bag"
# Open the bag file
bag = rosbag.Bag(bag_path)

# Initialize lists to store data
timestamps = []
data_values = []

timestamps_u = []
data_values_u = []

stepped = False
step_time = 0.0

# Iterate through the messages in the bag
for topic, msg, t in bag.read_messages():
    # Add your data processing and plotting code here
    # For example, if you have a specific topic containing sensor data:
    if topic == '/px150/joint_states':
        # Extract data from the message (replace with your actual message structure)
        if stepped == True:
            
            timestamp = msg.header.stamp.to_sec() - step_time
            data_value = msg.velocity[0]
            
            # Append data to lists
            timestamps.append(timestamp)
            
            data_values.append(data_value)
            
        else:
            timestamps.append(step_time)
            data_value = msg.velocity[0]
            data_values.append(data_value)
    #if topic == '/px150/waist_controller/command':
    if topic == '/px150/commands/joint_single':
        if stepped == False:
            step_time = t.to_sec()
            stepped = True
            
        timestamp = t.to_sec() - step_time # Convert Time to a float
        #print(timestamp)
        
        data_value = (msg.cmd * 0.113)/100 * 12

        # Append data to lists
        timestamps_u.append(timestamp)
        data_values_u.append(data_value)


max_value_index = data_values.index(max(data_values))
max_value = max(data_values)
max_timestamp = timestamps[max_value_index]
max_value_u = max(data_values_u)

# Find the value of 0.63 * max_value and its corresponding timestamp
threshold_value = 0.63 * max(data_values)

# Iterate through the data_values to find the first value greater than or equal to the threshold
for i in range(len(data_values)):
    if data_values[i] >= threshold_value:
        threshold_timestamp = timestamps[i]
        break
    
# Find the timestamp where data_values first become greater than 0    
first_positive_timestamp = None
for i in range(len(data_values)):
    if data_values[i] > 0:
        first_positive_timestamp = timestamps[i-1]
        break

tau = threshold_timestamp - first_positive_timestamp
Ko = max_value / 1.0

# Design parameters
w = 6
z = 1
a = 0.7
print("Open-Loop Impulse Response Measurements")
print("-----------------------------------------")
print("X_p_ss: ", max_value)
print("U: ", max_value_u)
print("Ko: ", Ko)
print("Tau: ", tau)
print("Tau_Lag: ", first_positive_timestamp)
print("\n")
Kp = ((w**2 + 2*z*w*a)*tau)/Ko #tau*w**2/Ko
Kd = ((2*z*w + a)*tau -1)/Ko #(2*z*w*tau - 1)/Ko
Ki = (w**2*a*tau)/Ko

print("PID Tuning Parameters")
print("-----------------------------------------")
print("Kp: ", Kp)
print("Kd: ", Kd)
print("Ki: ", Ki)


# Create subplots
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# Add a title to the entire figure
plt.suptitle('Step Response')

# Plot joint position on the first subplot
ax1.plot(timestamps, data_values, 'r-', label='Joint Velocity (rad/s)')
# Plot the maximum value
ax1.plot(max_timestamp, max_value, 'o', label='x_p_ss')
# Plot a dashed vertical line from threshold_timestamp to the threshold value
ax1.vlines(threshold_timestamp, ymin=0, ymax=threshold_value, color='b', linestyle='--', label='tau - Vertical Line')
# Plot a horizontal line from the beginning to the threshold timestamp at the height of the threshold value
ax1.hlines(threshold_value, xmin=timestamps[0], xmax=threshold_timestamp, color='b', linestyle='--', label='0.63 * X_p_ss')
ax1.set_xlim(0.0, 1.0)  # Set x-axis limits for the first subplot
ax1.set_ylabel('Joint Velocity (rad/s)')
ax1.legend()
ax1.grid(True)  # Turn on the grid for the first subplot

# Plot control signal on the second subplot
ax2.plot(timestamps_u, data_values_u, 'g-', label='Control Signal (v)')
ax2.set_xlim(0.0, 1.0)  # Set x-axis limits for the second subplot
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Control Signal (v)')
ax2.legend()
ax2.grid(True)  # Turn on the grid for the first subplot


# Show the plot
plt.show()

# Close the bag file when done
bag.close()
