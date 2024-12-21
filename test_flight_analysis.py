import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.signal import medfilt, savgol_filter

# Constants
mass = 5.42  # Mass of the rocket in kg
rho = 1.225  # Air density in kg/m^3
g = 9.81     # Acceleration due to gravity in m/s^2

# Load acceleration and velocity data 
accel_data = pd.read_csv("acceleration_data.csv")
velocity_data = pd.read_csv("velocity_data.csv") 
time_accel = accel_data['time'].values
acceleration_g = accel_data['acceleration'].values  # Acceleration in G
time_vel = velocity_data['time'].values
velocity_ft_s = velocity_data['velocity'].values  # Velocity in ft/s

# Convert acceleration to m/s² and velocity to m/s
acceleration_m_s2 = (acceleration_g-1.7) * g
velocity_m_s = velocity_ft_s * 0.3048

# Interpolate acceleration to match velocity times
interp_accel = interp1d(time_accel, acceleration_m_s2, kind='linear', fill_value='extrapolate')
acceleration_synced = interp_accel(time_vel)

# Remove gravity from acceleration
acceleration_drag = acceleration_synced - g

# Calculate C_d * A
velocity_squared = velocity_m_s**2
valid_indices = velocity_squared > 0  # Ignore points where velocity is zero
cd_a = np.zeros_like(velocity_m_s)
cd_a[valid_indices] = (-2 * mass * acceleration_drag[valid_indices]) / (rho * velocity_squared[valid_indices])

# Save drag acceleration and C_d*A to file
output_data = pd.DataFrame({
    'time': time_vel,
    'drag_acceleration': acceleration_drag,
    'cd_a': cd_a
})
output_data.to_csv('drag_and_cd_a.csv', index=False)

# Start at t >= 2 seconds and end at around 13 seconds
valid_time_indices = (time_vel >= 2)
time_vel_filtered = time_vel[valid_time_indices][:-80]
cd_a_filtered = cd_a[valid_time_indices][:-80]

# Apply median filter to remove spikes
cd_a_median_filtered = medfilt(cd_a_filtered, kernel_size=5)

# Apply Savitzky-Golay filter for smoothing
cd_a_smoothed = savgol_filter(cd_a_median_filtered, 7, 2)

# Plot results
plt.figure(figsize=(10, 6))
plt.plot(time_vel_filtered, cd_a_filtered, label='Raw $C_d \cdot A$', alpha=0.5, color='gray')
plt.plot(time_vel_filtered, cd_a_smoothed, label='Smoothed $C_d \cdot A$', color='blue')
plt.title('Drag Coefficient x Area vs Time (Smoothed)')
plt.xlabel('Time (s)')
plt.ylabel(r'$C_d \cdot A$ (m$^2$)')
plt.grid(True)
plt.legend()
plt.show()
