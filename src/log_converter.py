#!/usr/bin/env python
import time
import numpy as np
import rospy
import math as m
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scipy.ndimage.filters import gaussian_filter
import matplotlib.pyplot as plt
fig = plt.figure()

ahrs = []  # list to collect ahrs data from pixhawk
cerb = []  # list to collect ahrs data from cerberus (just calling it cerb) 
def odom_cb(msg):
    """
    odometry callback for pixhawk odometry message
    Params:
        message data
    Retruns:
        None
    """
    global ahrs
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (r,p,y) = euler_from_quaternion (orientation_list)
    ahrs.append([r*57.3,p*57.3,y*57.3])

def cerb_cb(msg):
    """
    odometry callback for cerberus odometry message
    Params:
        message data
    Retruns:
        None
    """
    global cerb
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (r,p,y) = euler_from_quaternion (orientation_list)
    cerb.append([r*57.3,p*57.3,y*57.3])

def wrap_180(ahrs):
    """
    This function prevents discontinuities in heading around pi/-pi. It makes the data into a single continuous curve.
    For instance, if the object rotated by 2 full rotations, it will show the final angle as being 720 degrees and not 0

    Params:
        ahrs data (roll, pitch, yaw)
    Returns:
        wrapped ahrs data (Same format as the input)
    """
    for i in range(len(ahrs)-1):
        if(m.fabs(ahrs[i+1,2]-ahrs[i,2])>170):
            ahrs[i+1:,2] -= (ahrs[i+1,2]-ahrs[i,2])
    return ahrs

odom_sub = rospy.Subscriber("odom/pixhawk",Odometry,odom_cb)  # subscriber for pixhawk/any other odometry publisher
cerb_sub = rospy.Subscriber("odom/cerberus",Odometry,cerb_cb)  # subscriber for cerberus odometry publisher
rospy.init_node("logger")  # init node
r = rospy.Rate(1)  # 1 second rate
while not rospy.is_shutdown():
    r.sleep()

ahrs = np.array(ahrs)  # convert list to numpy array 
cerb = np.array(cerb)

# rough estimate of the difference in orientation of pixhawk and cerberus module
ahrs[:,0] += 0.6
ahrs[:,1] += 0

time_cerb = np.arange(0,0.01*len(cerb), 0.01)
time_len_ahrs = 0.01*float(len(cerb))/float(len(ahrs)) # pixhawk gave data at a weird 5.5 Hz even though I set it to 50 Hz.
time = np.arange(0, time_len_ahrs*len(ahrs), time_len_ahrs)

# comparison of raw angles. I put this last in the blog for suspense reasons.
plt.suptitle("Cerberus vs Pixhawk absolute angle comparison", fontsize='x-large')
plt.subplot(311)
plt.title("Roll angle in degrees")
plt.plot(time,ahrs[:,0],label="Pixhawk Roll in degrees")
plt.plot(time_cerb,cerb[:,0],label="Cerberus Roll in degrees")
plt.xlabel("Time in seconds")
plt.ylabel("Angle in degrees")
plt.legend()

plt.subplot(312)
plt.title("Pitch angle in degrees")
plt.plot(time, -ahrs[:,1],label="Pixhawk pitch in degrees")
plt.plot(time_cerb,cerb[:,1],label="Cerberus pitch in degrees")
plt.xlabel("Time in seconds")
plt.ylabel("Angle in degrees")
plt.legend()

plt.subplot(313)
plt.title("Heading angle in degrees")
plt.plot(time,ahrs[:,2],label="Pixhawk Heading in degrees")
plt.plot(time_cerb,cerb[:,2],label="Cerberus Heading in degrees")
plt.xlabel("Time in seconds")
plt.ylabel("Angle in degrees")
plt.legend()
plt.show()

# wrap-around: helps with error measurment for yaw because yaw goes over 360 degrees and fucks up the error measurement
ahrs = wrap_180(ahrs)
cerb = wrap_180(cerb)
# gaussian filter with 1 sigma kernel to find the running average. anything greater and it rounds off too much.
running_avg_0 = gaussian_filter(ahrs[:,0], sigma = 2)
running_avg_1 = gaussian_filter(ahrs[:,1], sigma = 2)
running_avg_2 = gaussian_filter(ahrs[:,2], sigma = 2)

running_avg_cerb_0 = gaussian_filter(cerb[:,0], sigma = 2)
running_avg_cerb_1 = gaussian_filter(cerb[:,1], sigma = 2)
running_avg_cerb_2 = gaussian_filter(cerb[:,2], sigma = 2)

# fuck you pixhawk. Fuck you. having to clean up after your glitch.
ahrs_0_clean = (ahrs[:,0] - running_avg_0)[np.fabs(ahrs[:,0] - running_avg_0) < 1]
ahrs_1_clean = (ahrs[:,1] - running_avg_1)[np.fabs(ahrs[:,1] - running_avg_1) < 1]
ahrs_2_clean = (ahrs[:,2] - running_avg_2)[np.fabs(ahrs[:,2] - running_avg_2) < 1]

print(np.std(cerb[:,0] - running_avg_cerb_0), np.std(ahrs_0_clean))
print(np.std(cerb[:,1] - running_avg_cerb_1), np.std(ahrs_1_clean))
print(np.std(cerb[:,2] - running_avg_cerb_2), np.std(ahrs_2_clean))

signal = np.sin(2*m.pi*time_cerb)
noise = 0.02*np.sin(20*2*m.pi*time_cerb)
sigma_1 = gaussian_filter(signal + noise, sigma = 1)
sigma_2 = gaussian_filter(signal + noise, sigma = 2)
sigma_3 = gaussian_filter(signal + noise, sigma = 3)
plt.suptitle("sigma comparison")
plt.plot(time_cerb, signal,label="original")
plt.plot(time_cerb, sigma_1, label="1 sigma filtered")
plt.plot(time_cerb, sigma_2, label="2 sigma filtered")
plt.plot(time_cerb, sigma_3, label="3 sigma filtered")
plt.legend()
plt.show()

plt.suptitle("Pixhawk running error from gaussian filtered value in dynamic conditions", fontsize='x-large')
plt.subplot(311)
plt.title("Roll deviation in degrees")
plt.plot(time,ahrs[:,0] - running_avg_0,label="Roll error")
plt.plot(time,np.zeros_like(ahrs[:,0])+0.2,label="0.2 deg deviation line")
plt.plot(time,np.zeros_like(ahrs[:,0])-0.2,label="0.2 deg deviation line")
plt.plot(time,np.zeros_like(ahrs[:,0])+0.05,label="0.05 deg deviation line")
plt.plot(time,np.zeros_like(ahrs[:,0])-0.05,label="0.05 deg deviation line")
plt.xlabel("Time in seconds")
plt.ylabel("Angle error in degrees")
plt.legend()

plt.subplot(312)
plt.title("Pitch deviation in degrees")
plt.plot(time,ahrs[:,1] - running_avg_1,label="Pitch error")
plt.plot(time,np.zeros_like(ahrs[:,0])+0.2,label="0.2 deg deviation line")
plt.plot(time,np.zeros_like(ahrs[:,0])-0.2,label="0.2 deg deviation line")
plt.plot(time,np.zeros_like(ahrs[:,0])+0.05,label="0.05 deg deviation line")
plt.plot(time,np.zeros_like(ahrs[:,0])-0.05,label="0.05 deg deviation line")
plt.xlabel("Time in seconds")
plt.ylabel("Angle error in degrees")
plt.legend()

plt.subplot(313)
plt.title("Heading deviation in degrees")
plt.plot(time,ahrs[:,2] - running_avg_2,label="Heading error")
plt.plot(time,np.zeros_like(ahrs[:,0])+0.2,label="0.2 deg deviation line")
plt.plot(time,np.zeros_like(ahrs[:,0])-0.2,label="0.2 deg deviation line")
plt.plot(time,np.zeros_like(ahrs[:,0])+0.05,label="0.05 deg deviation line")
plt.plot(time,np.zeros_like(ahrs[:,0])-0.05,label="0.05 deg deviation line")
plt.xlabel("Time in seconds")
plt.ylabel("Angle error in degrees")
plt.legend()
plt.show()

plt.suptitle("Cerberus running error from gaussian filtered value in dynamic conditions", fontsize='x-large')
plt.subplot(311)
plt.title("Roll deviation in degrees")
plt.plot(time_cerb,cerb[:,0] - running_avg_cerb_0,label="Roll error")
plt.plot(time_cerb,np.zeros_like(cerb[:,0])+0.2,label="0.2 deg deviation line")
plt.plot(time_cerb,np.zeros_like(cerb[:,0])-0.2,label="0.2 deg deviation line")
plt.plot(time_cerb,np.zeros_like(cerb[:,0])+0.05,label="0.05 deg deviation line")
plt.plot(time_cerb,np.zeros_like(cerb[:,0])-0.05,label="0.05 deg deviation line")
plt.xlabel("Time in seconds")
plt.ylabel("Angle error in degrees")
plt.legend()

plt.subplot(312)
plt.title("Pitch deviation in degrees")
plt.plot(time_cerb,cerb[:,1] - running_avg_cerb_1,label="Pitch error")
plt.plot(time_cerb,np.zeros_like(cerb[:,0])+0.2,label="0.2 deg deviation line")
plt.plot(time_cerb,np.zeros_like(cerb[:,0])-0.2,label="0.2 deg deviation line")
plt.plot(time_cerb,np.zeros_like(cerb[:,0])+0.05,label="0.05 deg deviation line")
plt.plot(time_cerb,np.zeros_like(cerb[:,0])-0.05,label="0.05 deg deviation line")
plt.xlabel("Time in seconds")
plt.ylabel("Angle error in degrees")
plt.legend()

plt.subplot(313)
plt.title("Heading deviation in degrees")
plt.plot(time_cerb,cerb[:,2] - running_avg_cerb_2,label="Heading error")
plt.plot(time_cerb,np.zeros_like(cerb[:,0])+0.2,label="0.2 deg deviation line")
plt.plot(time_cerb,np.zeros_like(cerb[:,0])-0.2,label="0.2 deg deviation line")
plt.plot(time_cerb,np.zeros_like(cerb[:,0])+0.05,label="0.05 deg deviation line")
plt.plot(time_cerb,np.zeros_like(cerb[:,0])-0.05,label="0.05 deg deviation line")
plt.xlabel("Time in seconds")
plt.ylabel("Angle error in degrees")
plt.legend()

plt.show()