import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.ndimage.filters import gaussian_filter
import math as m
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

log_file = 'INS_log_2021_3_9_9_23.npy'
data = np.load(log_file,allow_pickle=True)

data = data[:-150]
state = data[:,0:13]
quat = state[:,0:4]
print(state.shape)
Cov = data[:,13:17]
max_exec_time = data[:,17]
gps_data = data[:,18:27]
ADS = data[:,27:29]
IMU = data[:,29:38]
OPFlow = data[:,38:42]

def quat2eul(u):
	y = np.zeros(3)
	y[0] = m.atan2((2.0*(u[2]*u[3]+u[0]*u[1])) , (u[0]*u[0]-u[1]*u[1]-u[2]*u[2]+u[3]*u[3]));
	y[1] = -m.asin(2.0*(u[1]*u[3]-u[0]*u[2]));
	y[2] = m.atan2((2.0*(u[1]*u[2]+u[0]*u[3])) , (u[0]*u[0]+u[1]*u[1]-u[2]*u[2]-u[3]*u[3]));
	return y

time = np.arange(0,len(max_exec_time)*0.02,0.02)
# print(time.shape,state[:,9].shape)
ins_yaw = np.zeros_like(time)
imu_yaw = state[:,12]

# for i in range(len(time)):
# 	ins_yaw[i] = quat2eul(state[i,:4])[2]
# 	if(imu_yaw[i]>m.pi):
# 		imu_yaw[i] -= 2*m.pi


def calcposNED(lat, lon, hgt, latReference, lonReference, hgtReference):
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


# plt.plot(time, gaussian_filter(IMU[:,0],sigma=1),label="gx")
# plt.plot(time, IMU[:,2]+9.8,label="gy")
# plt.plot(time, IMU[:,5],label="gz")
# plt.plot(time, 57.3*ins_yaw,label="ins heading")
# plt.plot(time, 57.3*imu_yaw,label="imu heading")
# ax.axis('equal')
# print(state)
# ax.plot(state[:,8],state[:,7],zs = -state[:,9], zdir='z',label="filtered XYZ position")
# ax.plot(np.zeros(10),np.zeros(10),-100*np.ones(10))
# plt.plot(time,gps_data[:,2]-gps_data[0,2],label='gps')
# plt.plot(time[:-1],np.diff(-state[:,6]),label='filt_v')
# plt.plot(time[:-1], np.diff(ADS[:,1]),label='baro_v')
# plt.plot(time,-gps_data[:,5],label='gps_d')
# plt.plot(time,np.cumsum(np.cumsum(IMU[:,2]+9.865)*0.02)*0.02,label='Accel_Z')
# plt.plot(time,ADS[:,1]-ADS[0,1],label='baro')
# plt.plot(time,gps_data[:,7],label='gps_sAcc')
# plt.plot(time,max_exec_time)
posNEDr = calcposNED(gps_data[:,0],gps_data[:,1],gps_data[:,2],gps_data[0,0],gps_data[0,1],gps_data[0,2])
# plt.plot(state[:,8],state[:,7],label='filt')
ax.plot(posNEDr[:,1],posNEDr[:,0],zs = gps_data[:,2]-gps_data[0,2] ,label='gps')

# ax.plot(state[:,8],state[:,7],zs = gps_data[:,2]-gps_data[0,2], zdir='z',label="filtered XY, gps_alt")
ax.plot(state[:,8],state[:,7],zs=-state[:,9], zdir='z',label="filtered XYZ")
ax.plot(state[:,8],state[:,7],zs = ADS[:,1]-ADS[0,1], zdir='z',label="filtered XY, baro")
# ax.plot(state[:,8],state[:,7],zs = np.ones_like(state[:,9])*2.5,zdir='z',label='baseline')

plt.legend()
plt.axis('equal')
# ax.plot(state[:,8],state[:,7],zs = ADS[:,1]-ADS[0,1],zdir='z',label="filtered XY, baro Z")
# ax.plot(np.cumsum(gps_data[:,4])*0.2,np.cumsum(gps_data[:,3])*0.2,np.cumsum(-gps_data[:,5])*0.2,label="integrated")
# ax.plot(state[:,8],state[:,7],zs = IMU[:,2]+9.8, zdir='z',label="filtered XYZ position")

plt.show()
