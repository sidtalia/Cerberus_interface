import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.ndimage.filters import gaussian_filter
import math as m
import os
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

bags_dir = os.path.abspath(os.path.join(os.getcwd(), os.pardir)) + '/bags/'
log_file = 'INS_log_2021_6_11_16_14.npy' #bags_dir + 'INS_log_random_angle.npy'
data = np.load(log_file,allow_pickle=True)

state = data[:,0:13] # state information
quat = state[:,0:4]  # orientation represented in quaternions.
Cov = data[:,13:17]  # contains dummy data
max_exec_time = data[:,17]  # maximum code execution time
gps_data = data[:,18:27]  # gps lat, lon, alt, horizontal accuracy, vertical accuracy, speed accuracy
ADS = data[:,27:29]  # air-data (barometer height, airspeed (not available or represents number of satalites in some later logs))
IMU = data[:,29:38]  # IMU sensor data, accelerometer, gyroscope, magnetometer
OPFlow = data[:,38:42]  # optical flow sensor data, not used and either set to 0 or some random value

def quat2eul(u):
    """
    function to convert quaternion angles to euler angles.

    Params: 
        u: quaternion (x,y,z,w)
    Returns:
        y: euler angles as a numpy array (roll, pitch, yaw) 
    """
    y = np.zeros(3)
    y[0] = m.atan2((2.0*(u[2]*u[3]+u[0]*u[1])) , (u[0]*u[0]-u[1]*u[1]-u[2]*u[2]+u[3]*u[3]));
    y[1] = -m.asin(2.0*(u[1]*u[3]-u[0]*u[2]));
    y[2] = m.atan2((2.0*(u[1]*u[2]+u[0]*u[3])) , (u[0]*u[0]+u[1]*u[1]-u[2]*u[2]-u[3]*u[3]));
    return y

time = np.arange(0,len(max_exec_time)*0.02,0.02)

ahrs = state[:,10:]*57.29 # the AHRS data is in radians. Need to convert to degrees
time = np.arange(0.0,len(ahrs)*0.01,0.01)  # 100 Hz data rate. converting to seconds.
time = time[:len(ahrs)]  # just making sure that the lengths of both are the same

def calcposNED(lat, lon, hgt, latReference, lonReference, hgtReference):
    """
    function for calculating NED relative position between two lat-lon-alt coordinates
    Params:
        lat,lon,hgt, reference lat, reference lon, reference hgt
    Returns:
        posNED relative in meters as a numpy array (north distance, east distance, down distance)
    """
    earthRadius = 6378145.0
    posNEDr = np.zeros((len(lat),3))
    lat /= 57.3
    lon /= 57.3
    latReference /= 57.3
    lonReference /= 57.3
    print(latReference)
    posNEDr[:,0] = earthRadius * (lat - latReference)
    posNEDr[:,1] = earthRadius * np.cos(latReference) * (lon - lonReference)
    posNEDr[:,2] = -(hgt - hgtReference)
    return posNEDr

start_stamp = 0  # in case you want to start at a different time-stamp.
# print standard deviation of static data
print(np.std(ahrs[start_stamp:,0]))  # roll
print(np.std(ahrs[start_stamp:,1]))  # pitch
print(np.std(ahrs[start_stamp:,2]))  # heading

plt.subplot(311)
plt.title("Roll angle stability across time")
plt.plot(time[start_stamp:],ahrs[start_stamp:,0],label="Roll in degrees")
plt.plot(time[start_stamp:],np.ones_like(time[start_stamp:])*np.mean(ahrs[start_stamp:,0])+0.05,label="0.05 deg deviation line")
plt.plot(time[start_stamp:],np.ones_like(time[start_stamp:])*np.mean(ahrs[start_stamp:,0])-0.05,label="0.05 deg deviation line")
plt.xlabel("Time in seconds")
plt.ylabel("Angle in degrees")
plt.legend()
plt.subplot(312)
plt.title("Pitch angle stability across time")
plt.plot(time[start_stamp:],ahrs[start_stamp:,1],label="Pitch in degrees")
plt.plot(time[start_stamp:],np.ones_like(time[start_stamp:])*np.mean(ahrs[start_stamp:,1])+0.05,label="0.05 deg deviation line")
plt.plot(time[start_stamp:],np.ones_like(time[start_stamp:])*np.mean(ahrs[start_stamp:,1])-0.05,label="0.05 deg deviation line")
plt.xlabel("Time in seconds")
plt.ylabel("Angle in degrees")
plt.legend()
plt.subplot(313)
plt.title("Heading angle stability across time")
plt.plot(time[start_stamp:],ahrs[start_stamp:,2],label="Heading in degrees")
plt.plot(time[start_stamp:],np.ones_like(time[start_stamp:])*np.mean(ahrs[start_stamp:,2])+0.5, label="0.2 deg deviation line")
plt.plot(time[start_stamp:],np.ones_like(time[start_stamp:])*np.mean(ahrs[start_stamp:,2])-0.5, label="0.2 deg deviation line")
plt.xlabel("Time in seconds")
plt.ylabel("Angle in degrees")

plt.legend()

plt.show()
