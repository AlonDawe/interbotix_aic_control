import rosbag
import matplotlib.pyplot as plt
from matplotlib import rcParams

# Set font and size to match LaTeX document
#rcParams['font.family'] = 'serif'
#rcParams['font.serif'] = ['Computer Modern']
#rcParams['font.size'] = 10
rcParams['font.family'] = 'serif'
rcParams['font.serif'] = ['cmr10']
rcParams['font.size'] = 14
#rcParams['axes.unicode_minus'] = False
rcParams['axes.formatter.use_mathtext'] = True

# Specify the path to your ROS bag file
#bag_path = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/_2023-11-21-14-13-48.bag"
bag_path = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/waist/WAIST_STEP_2023-12-04-11-24-48.bag"
#bag_path = "/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/waist/WAIST_STEP_2024-03-13-10-06-24.bag"
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
        
        data_value = (msg.cmd * 0.113)/100 * 12 # --> PWM * 0.113 % of 12V 

        # Append data to lists
        timestamps_u.append(timestamp)
        data_values_u.append(data_value)
        
print(len(data_values))
print(len(data_values_u))

data_values_u.append(0.0)
timestamps_u.append(1.0)

#print(len())

#print(data_values_u)

#while len(data_values) != len(data_values_u):
#    data_values_u.append(0.0)
    
#print(data_values_u)

max_value_index = data_values.index(max(data_values))
max_value = max(data_values)
max_timestamp = timestamps[max_value_index]
max_value_u = max(data_values_u)

# Find the value of 0.63 * max_value and its corresponding timestamp

threshold_value = 0.63 * max(data_values)
final_threshold_value = 0.0

#print("threshold value = ", threshold_value)

# Iterate through the data_values to find the first value greater than or equal to the threshold
for i in range(len(data_values)):
    if data_values[i] >= threshold_value:
        threshold_timestamp = timestamps[i-1]
        final_threshold_value = data_values[i-1]
        break
print("threshold timestamp = ", threshold_timestamp)
print("final threshold value = ", final_threshold_value)
    
# Find the timestamp where data_values first become greater than 0    
first_positive_timestamp = None
for i in range(len(data_values)):
    if data_values[i] > 0:
        first_positive_timestamp = timestamps[i-1]
        break

tau = threshold_timestamp - first_positive_timestamp
Ko = max_value / 0.339 #0.508max_value

# Design parameters
w = 4.09#4.3
z = 0.61 #0.9
a = 10.77 #1.7
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

Kp = ((w**2 + 0)*tau)/Ko #tau*w**2/Ko
Kd = ((2*z*w + 0)*tau -1)/Ko #(2*z*w*tau - 1)/Ko

print("PD Tuning Parameters")
print("-----------------------------------------")
print("Kp: ", Kp)
print("Kd: ", Kd)



# Create subplots
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# Add a title to the entire figure
#plt.suptitle('Open-Loop Impulse Response')

# Plot joint position on the first subplot
ax1.step(timestamps, data_values, label='Waist Joint')
# Plot the maximum value
ax1.plot(max_timestamp, max_value, 'o', label='$X_{p} = 1.918$')
# Plot a dashed vertical line from threshold_timestamp to the threshold value
ax1.vlines(threshold_timestamp, ymin=0, ymax=final_threshold_value, color='r', linestyle='--', label=r'$\tau + t_{d}$ = 0.121')
# Plot a horizontal line from the beginning to the threshold timestamp at the height of the threshold value
#ax1.hlines(final_threshold_value, xmin=timestamps[0], xmax=threshold_timestamp, color='r', linestyle='--', label=r' $\tau = 0.63 X_{p}$')
ax1.set_xlim(0, 1.0)  # Set x-axis limits for the first subplot
ax1.set_ylabel('Joint Velocity (rad/s)')
#ax1.legend()
ax1.legend(ncol=3, fontsize='small', frameon=False, loc='upper center', bbox_to_anchor =(0.5, 1.28))
ax1.grid(True)  # Turn on the grid for the first subplot

ax1.annotate('', xy=(threshold_timestamp, -0.05), xytext=(first_positive_timestamp, -0.05), arrowprops=dict(arrowstyle='<->'))
ax1.text((first_positive_timestamp + threshold_timestamp) / 2, -0.08, r'$\tau$', ha='center', va='top')

# Add arrow from 0 to t_d and label it t_d
#ax1.annotate('', xy=(first_positive_timestamp, -0.05), xytext=(0, -0.05), arrowprops=dict(arrowstyle='<->'))
ax1.text(first_positive_timestamp / 1.5, -0.08, r'$t_{d}$', ha='center', va='top')


# Plot control signal on the second subplot
[0.4, 1.0]
ax2.step(timestamps_u, data_values_u)
#ax2.plot([0.4, 1.0], [0.0, 0.0])[0.4, 1.0]
ax2.set_xlim(0, 1.0)  # Set x-axis limits for the second subplot
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Control Signal (V)')
#ax2.legend()
ax2.grid(True)  # Turn on the grid for the first subplot

fig.savefig('/home/alon/Documents/thesis/Tuning/OL_impulse_response.pdf')


# Show the plot
plt.show()

# Close the bag file when done
bag.close()

from scipy.optimize import fsolve

# Given PID values

Kp = 1.1732
Kd = 0.0852
Ki = 3.0

# Given PD values
#Kp = 1.0413
#d = 0.0888
#Ki = 0.0

#Ko = 5.65919
#tau = 0.09399

# Define the equations PID
def equations(vars):
    w, a, z = vars
    eq1 = (w**2 + 2*z*w*a)*tau/Ko - Kp
    eq2 = ((2*z*w + a)*tau - 1)/Ko - Kd
    eq3 = w**2*a*tau/Ko - Ki
    return [eq1, eq2, eq3]

def equationsPD(vars):
    w, z = vars
    eq1 = (w**2)*tau/Ko - Kp
    eq2 = ((2*z*w)*tau - 1)/Ko - Kd
    return [eq1, eq2]

# Initial guess
guess = [1, 1, 1]
guessPD = [1, 1]

# Solve the equations
w, a, z = fsolve(equations, guess)

print("PID CONTROLLER")
print("w =", w)
print("a =", a)
print("z =", z)
print("\n")

#Given PD values
Kp = 1.0413
d = 0.0888
Ki = 0.0

w, z = fsolve(equationsPD, guessPD)
print("PD CONTROLLER")
print("w =", w)
print("z =", z)


