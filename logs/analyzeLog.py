
# Import pandas 
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import argparse

# Parse arguments
parser = argparse.ArgumentParser(description='Log analysis and plotting')
parser.add_argument('logfile', type=str, help='Log file to analyze')
args = parser.parse_args()

# Check if log directory and plot directory exists
log_dir = "flight_logs"
if not os.path.exists(log_dir):
    os.mkdir(log_dir)
plot_dir = "plots"
if not os.path.exists(plot_dir):
    os.mkdir(plot_dir)

# Read the log fils
filename = os.path.join(log_dir, args.logfile)
# Start at row 31 and ignore all parameters for now
data = pd.read_csv(filename, header=31) 

# Print data types
print("Data types:\n", data.dtypes)



# Create a time vector replacing the nans with the mean dt
dt_mean = np.mean(np.nan_to_num(data["dt"].to_numpy(), nan=0))
logtime = np.nan_to_num(data["dt"].to_numpy(), nan=dt_mean)
logtime = np.divide(np.cumsum(logtime),1000.0)



# Plot attitude
plt.figure()
plt.subplot(221)
plt.plot(logtime, data["roll"].to_numpy())
plt.grid(True)
plt.xlabel('Time [sec]')
plt.ylabel('Roll [deg]')

plt.subplot(222)
plt.plot(logtime, data["pitch"].to_numpy())
plt.grid(True)
plt.xlabel('Time [sec]')
plt.ylabel('Pitch [deg]')

plt.subplot(223)
plt.plot(logtime, data["yaw"].to_numpy())
plt.grid(True)
plt.xlabel('Time [sec]')
plt.ylabel('Yaw [deg]')

plt.subplot(224)
plt.plot(logtime, data["roll"].to_numpy(), 'r')
plt.plot(logtime, data["pitch"].to_numpy(), 'g')
plt.plot(logtime, data["yaw"].to_numpy(), 'b')
plt.grid(True)
plt.xlabel('Time [sec]')
plt.ylabel('All [deg]')

plt.subplots_adjust(top=0.92, bottom=0.08, left=0.10, right=0.95, hspace=0.25, wspace=0.35)
plt.savefig(os.path.join(plot_dir,"attitude.png"))


# Plot Baro height
plt.figure()
plt.subplot(221)
plt.plot(logtime, np.divide(data["alt"].to_numpy(),1000))
plt.grid(True)
plt.xlabel('Time [sec]')
plt.ylabel('Alt [m]')

plt.subplot(222)
plt.plot(logtime, np.divide(data["zvel"].to_numpy(),1000))
plt.grid(True)
plt.xlabel('Time [sec]')
plt.ylabel('z vel [m/s]')

plt.subplot(223)
plt.plot(logtime, np.divide(data["zacc"].to_numpy(),1000))
plt.grid(True)
plt.xlabel('Time [sec]')
plt.ylabel('z acc [m/s^2]')

plt.subplot(224)
plt.plot(logtime, np.divide(data["altLPF"].to_numpy(),1000))
plt.grid(True)
plt.xlabel('Time [sec]')
plt.ylabel('Alt LPF [m]')

plt.subplots_adjust(top=0.92, bottom=0.08, left=0.10, right=0.95, hspace=0.25, wspace=0.35)
plt.savefig(os.path.join(plot_dir,"baroHeight.png"))


# Sonar height
plt.figure()
plt.subplot(211)
plt.plot(logtime, np.divide(data["sonarDist"].to_numpy(),1000))
plt.grid(True)
plt.xlabel('Time [sec]')
plt.ylabel('sonarDist [m]')

plt.subplot(212)
plt.plot(logtime, np.divide(data["dist"].to_numpy(),1000))
plt.grid(True)
plt.xlabel('Time [sec]')
plt.ylabel('dist [m]')
plt.savefig(os.path.join(plot_dir,"sonarHeight.png"))
