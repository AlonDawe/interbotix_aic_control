import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D, art3d
from matplotlib.patches import Circle
from matplotlib import rcParams
from matplotlib import cm

rcParams['font.family'] = 'serif'
rcParams['font.serif'] = ['cmr10']
rcParams['font.size'] = 14
rcParams['text.usetex'] = True
#rcParams['axes.unicode_minus'] = False
rcParams['axes.formatter.use_mathtext'] = True
# Set font and size to match LaTeX document
#rcParams['font.family'] = 'serif'
#rcParams['font.serif'] = ['Computer Modern']
#rcParams['font.size'] = 10

Kp1 = np.load('/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/parameter_grid_Kp_1.npy') # Kp1 = [ka, step, performance_metric]
Kp2 = np.load('/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/parameter_grid_Kp_2.npy')
Kp3 = np.load('/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/parameter_grid_Kp_3.npy')
Kp4 = np.load('/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/parameter_grid_Kp_4.npy')
Kp5 = np.load('/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/parameter_grid_Kp_5.npy')
Kp6 = np.load('/home/alon/ros_workspaces/interbotix_pincherX_ws/src/interbotix_aic_control/bagfiles/tuning/parameter_grid_Kp_6.npy')

# Specify the file names for saving CSV files
csv_file_Kp1 = 'Kp1_1.csv'
csv_file_Kp2 = 'Kp2_1.csv'
csv_file_Kp3 = 'Kp3_1.csv'
csv_file_Kp4 = 'Kp4_1.csv'
csv_file_Kp5 = 'Kp5_1.csv'
csv_file_Kp6 = 'Kp6_1.csv'

# Save NumPy arrays as CSV files
#np.savetxt(csv_file_Kp1, Kp1[:, 6, :], delimiter=',')
#np.savetxt(csv_file_Kp2, Kp2[:, 6, :], delimiter=',')
#np.savetxt(csv_file_Kp3, Kp3[:, 6, :], delimiter=',')
#np.savetxt(csv_file_Kp4, Kp4[:, 6, :], delimiter=',')
#np.savetxt(csv_file_Kp5, Kp5[:, 6, :], delimiter=',')
#np.savetxt(csv_file_Kp6, Kp6[:, 6, :], delimiter=',')


loaded_array = np.array([Kp1[:, 0, :], Kp2[:, 0, :], Kp3[:, 0, :], Kp4[:, 0, :], Kp5[:, 0, :], Kp6[:, 0, :]])

loaded_array_step_1 = np.array([Kp1[:, 0, :], Kp2[:, 0, :], Kp3[:, 0, :], Kp4[:, 0, :], Kp5[:, 0, :], Kp6[:, 0, :]])
loaded_array_step_2 = np.array([Kp1[:, 1, :], Kp2[:, 1, :], Kp3[:, 1, :], Kp4[:, 1, :], Kp5[:, 1, :], Kp6[:, 1, :]])
loaded_array_step_3 = np.array([Kp1[:, 2, :], Kp2[:, 2, :], Kp3[:, 2, :], Kp4[:, 2, :], Kp5[:, 2, :], Kp6[:, 2, :]])
loaded_array_step_4 = np.array([Kp1[:, 3, :], Kp2[:, 3, :], Kp3[:, 3, :], Kp4[:, 3, :], Kp5[:, 3, :], Kp6[:, 3, :]])
loaded_array_step_5 = np.array([Kp1[:, 4, :], Kp2[:, 4, :], Kp3[:, 4, :], Kp4[:, 4, :], Kp5[:, 4, :], Kp6[:, 4, :]])
loaded_array_step_6 = np.array([Kp1[:, 5, :], Kp2[:, 5, :], Kp3[:, 5, :], Kp4[:, 5, :], Kp5[:, 5, :], Kp6[:, 5, :]])
loaded_array_step_7 = np.array([Kp1[:, 6, :], Kp2[:, 6, :], Kp3[:, 6, :], Kp4[:, 6, :], Kp5[:, 6, :], Kp6[:, 6, :]])

loaded_arrays = [loaded_array_step_1, loaded_array_step_2, loaded_array_step_3, loaded_array_step_4, loaded_array_step_5, loaded_array_step_6, loaded_array_step_7]

# Convert the list of arrays into a NumPy array
stacked_arrays = np.stack(loaded_arrays)


# Calculate the mean across the first axis (which represents the different steps)
averaged_array = np.mean(stacked_arrays, axis=0)
#print(averaged_array)
# Calculate the variance across the first axis (which represents the different steps)
final_variance = np.var(stacked_arrays, axis=0)
# Calculate the standard deviation across the first axis (which represents the different steps)
final_std = np.std(stacked_arrays, axis=0)

# Find the minimum mean of the multidimensional array
min_value_axis_0 = np.min(averaged_array[:, :, 0])
min_value_axis_1 = np.min(averaged_array[:, :, 1])
min_value_axis_2 = np.min(averaged_array[:, :, 2])

zero_indices = np.where(averaged_array[:, :, 2] == min_value_axis_2)

print(zero_indices)

min_values = [min_value_axis_0, min_value_axis_1, min_value_axis_2]




min_indices_axis_0 = np.unravel_index(np.argmin(averaged_array[:, :, 0]), averaged_array[:, :, 0].shape)
min_indices_axis_1 = np.unravel_index(np.argmin(averaged_array[:, :, 1]), averaged_array[:, :, 1].shape)
min_indices_axis_2 = np.unravel_index(np.argmin(averaged_array[:, :, 2]), averaged_array[:, :, 2].shape)


min_indices = [min_indices_axis_0, min_indices_axis_1, min_indices_axis_2]

print()

print("ITAE")
print(min_value_axis_0)
print("Kp: ", min_indices_axis_0[0]+1)
print("Ka: ", min_indices_axis_0[1]+1)

print("\n")

print("ST")
print(min_value_axis_1)
print("Kp: ", min_indices_axis_1[0]+1)
print("Ka: ", min_indices_axis_1[1]+1)

print("\n")

print("OS")
print(min_value_axis_2)
print("Kp: ", min_indices_axis_2[0]+1)
print("Ka: ", min_indices_axis_2[1]+1)





#print(Kp1[0])

# Define Kp and Ka ranges
kp_values = np.linspace(1, 5, averaged_array.shape[1])
ka_values = np.linspace(1, 6, averaged_array.shape[0])

# Create meshgrid for the custom X and Y axes
kp_mesh, ka_mesh = np.meshgrid(kp_values, ka_values)

performance_metric = ["ITAE Score", "Settling Time ($sec$)", "Overshoot ($\%$)"]

# Create a single figure with three subplots arranged in a row
fig, axes = plt.subplots(1, 3, figsize=(11, 3.5), subplot_kw={'projection': '3d'})

for i, ax in enumerate(axes):
    surf = ax.plot_surface(kp_mesh, ka_mesh, averaged_array[:, :, i], cmap='viridis', zorder=1)
    # Set labels
    ax.set_xlabel('$\kappa_{a}$')
    ax.set_ylabel('$\mathcal{K}_{p}$')
    ax.zaxis.set_rotate_label(False)  # disable automatic rotation
    ax.set_zlabel(performance_metric[i] , rotation=90)
    ax.set_title(performance_metric[i])
    #ax.view_init(elev=15, azim=60 * i - 45)  # Adjust the elevation and azimuth angles
    ax.view_init(elev=30, azim=15)  # Adjust the azimuth angle
    
    
  
#fig.suptitle('Mean Performance Metric Analysis', fontsize=16)
#fig.savefig('/home/alon/Documents/thesis/Tuning Parameter Influences/mean_performance_metrics.pdf')

performance_metric = ["ITAE Score", "Settling Time ($sec$)", "Overshoot ($\%$)"]
fig2, axes = plt.subplots(1, 3, figsize=(11, 3.5), subplot_kw={'projection': '3d'})

for i, ax in enumerate(axes):
    # Plot the 3D surface for each performance metric
    surf = ax.plot_surface(kp_mesh, ka_mesh, final_std[:, :, i], cmap='viridis')

    # Set labels
    ax.set_xlabel('$\kappa_{a}$')
    ax.set_ylabel('$\mathcal{K}_{p}$')
    ax.zaxis.set_rotate_label(False)  # disable automatic rotation
    ax.set_zlabel(performance_metric[i] , rotation=90)
    ax.set_title(performance_metric[i])
    #ax.view_init(elev=15, azim=60 * i - 45)  # Adjust the elevation and azimuth angles
    ax.view_init(elev=30, azim=15)  # Adjust the azimuth angle
    
    
  
#fig2.suptitle('Variance Performance Metric Analysis', fontsize=16)
#fig2.savefig('/home/alon/Documents/thesis/Tuning Parameter Influences/std_performance_metrics.pdf')

#performance_metric = ["ITAE", "Settling Time (s)", "Overshoot (%)"]
performance_metric = ["ITAE", "$t_{s}$ (s)", "$M_{p}$ (\%)"]
performance_metric_name = ["ITAE_Score", "Settling_Time", "Overshoot"]
plt.rcParams['grid.color'] = "whitesmoke"
for i in range(3):

    # Create a 3D plot for loaded_array[:, :, 0]
    fig = plt.figure(figsize=(3.5, 3.5))
    ax = fig.add_subplot(111, projection='3d')
    # Get rid of the panes
    ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    
    
    
    #ax.grid(False)
    
    ax.grid(alpha = 0.9)
    
    norm = plt.Normalize(averaged_array[:, :, i].min(), averaged_array[:, :, i].max())
    colors = cm.copper(norm(averaged_array[:, :, i]))
    rcount, ccount, _ = colors.shape
    
    # Plot the 3D surface
    surf = ax.plot_surface(kp_mesh, ka_mesh, averaged_array[:, :, i], rcount=rcount, ccount=ccount,
                       facecolors=colors, shade=False, zorder=0)
    surf.set_facecolor((0,0,0,0))
    #ax.grid(color='whitesmoke')
    #ax.plot_wireframe(kp_mesh, ka_mesh, averaged_array[:, :, i], cmap='viridis')
    # Set labels
    if i != 2:
        ax.scatter((min_indices[i][1]+1), (min_indices[i][0]+1), min_values[i], marker='o', color=(0, 0.678, 0.937), s=35, zorder=1, depthshade=False)
    else:
        ax.scatter((zero_indices[1]+1), (zero_indices[0]+1), min_values[i], marker='o', color=(0, 0.678, 0.937), s=35, zorder=1, depthshade=False)
    ax.set_xlabel('$\kappa_{a}$')
    ax.set_ylabel('$\mathcal{K}_{p}$')
    ax.zaxis.set_rotate_label(False)  # disable automatic rotation
    ax.set_zlabel(performance_metric[i], rotation=90)
    #ax.set_title(performance_metric[i])
    ax.view_init(elev=30, azim=15)  # Adjust the azimuth angle
    #fig.savefig('/home/alon/Documents/thesis/Tuning Parameter Influences/mean_performance_metrics_'+performance_metric_name[i]+'_new'+'.pdf')
    
for i in range(3):

    # Create a 3D plot for loaded_array[:, :, 0]
    fig = plt.figure(figsize=(3.5, 3.5))
    ax = fig.add_subplot(111, projection='3d')
    ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    norm = plt.Normalize(final_std[:, :, i].min(), final_std[:, :, i].max())
    colors = cm.copper(norm(final_std[:, :, i]))
    rcount, ccount, _ = colors.shape

    # Plot the 3D surface
    surf = ax.plot_surface(kp_mesh, ka_mesh, final_std[:, :, i], rcount=rcount, ccount=ccount,
                       facecolors=colors, shade=False, zorder=0)
    surf.set_facecolor((0,0,0,0))
    #ax.plot_wireframe(kp_mesh, ka_mesh, final_std[:, :, i], cmap='viridis')
    # Set labels
    ax.set_xlabel('$\kappa_{a}$')
    ax.set_ylabel('$\mathcal{K}_{p}$')
    ax.zaxis.set_rotate_label(False)  # disable automatic rotation
    ax.set_zlabel(performance_metric[i], rotation=90)
    #ax.set_title(performance_metric[i])
    ax.view_init(elev=30, azim=15)  # Adjust the azimuth angle
    #fig.savefig('/home/alon/Documents/thesis/Tuning Parameter Influences/std_performance_metrics_'+performance_metric_name[i]+'_new'+'.pdf')

# Show the plot
plt.show()