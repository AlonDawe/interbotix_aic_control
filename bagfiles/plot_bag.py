import rosbag
import matplotlib.pyplot as plt

# Specify the path to your ROS bag file
bag_path = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/_2023-11-15-10-13-41.bag"

# Open the bag file
bag = rosbag.Bag(bag_path)

# Initialize lists to store data
timestamps = []
data_values = []

timestamps_u = []
data_values_u = []

# Iterate through the messages in the bag
for topic, msg, t in bag.read_messages():
    # Add your data processing and plotting code here
    # For example, if you have a specific topic containing sensor data:
    if topic == '/px150/joint_states':
        # Extract data from the message (replace with your actual message structure)
        timestamp = msg.header.stamp.to_sec()
        data_value = msg.velocity[5]

        # Append data to lists
        timestamps.append(timestamp)
        data_values.append(data_value)
    if topic == '/px150/waist_controller/command':
        timestamp = t.to_sec()  # Convert Time to a float
        data_value = msg.data

        # Append data to lists
        timestamps_u.append(timestamp)
        data_values_u.append(data_value)

print(timestamps_u[0])       
# Plot the data using a solid red line
plt.plot(timestamps, data_values, 'r-', label='/px150/joint_states')
plt.plot(timestamps_u, data_values_u, 'g-', label='/px150/waist_controller/command')
        

# Add labels and legend if necessary
plt.grid(True)
plt.xlabel('Timestamp')
plt.ylabel('Data Value')
plt.legend()
plt.title('ROS Bag Data Plot')

max_value_index = data_values.index(max(data_values))
max_value = max(data_values)
max_timestamp = timestamps[max_value_index]

max_value_u = max(data_values_u)

plt.plot(max_timestamp, max_value, 'o', label='x_p_ss')

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
Ko = max_value / max_value_u

# Design parameters
w = 40
z = 1
print("Open-Loop Impulse Response Measurements")
print("-----------------------------------------")
print("Ko: ", Ko)
print("Tau: ", tau)
print("\n")
Kp = tau*w**2/Ko
Kd = (2*z*w*tau - 1)/Ko

print("PID Tuning Parameters")
print("-----------------------------------------")
print("Kp: ", Kp)
print("Kd: ", Kd)
    
# Plot a dashed vertical line from threshold_timestamp to the threshold value
plt.vlines(threshold_timestamp, ymin=0, ymax=threshold_value, color='b', linestyle='--', label='Threshold Line')
# Plot a horizontal line from the beginning to the threshold timestamp at the height of the threshold value
plt.hlines(threshold_value, xmin=timestamps[0], xmax=threshold_timestamp, color='b', linestyle='--', label='Horizontal Line')


# Show the plot
plt.show()

# Close the bag file when done
bag.close()
