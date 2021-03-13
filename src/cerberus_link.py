#!/usr/bin/env python
import time
import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math as m 
import traceback
from DAVID_link import *



ins_link = INS(COM='/dev/ttyUSB0',baud = 921600)

imu_quat = None
A = np.zeros(3)
G = np.zeros(3)
def vel_BF_cb(data):
	global ins_link
	vx = data.twist.twist.linear.x
	vy = data.twist.twist.linear.y
	vz = data.twist.twist.linear.z
	# TODO: replace with something to convert vio from any orientation to any orientation.
	th = 20/57.3
	V = np.zeros(4,dtype='float32')
	V[0] = -vy
	V[1] = vx*m.cos(th) - vz*m.sin(th)
	V[2] = vx*m.sin(th) + vz*m.cos(th)
	V[3] = data.twist.covariance[0]
	ins_link.send_vel_BF(V) # body frame xyz is front right down

def pos_cb(data):
	global ins_link
	P = np.zeros(4,dtype='float32')
	P[0] = data.pose.position.x
	P[1] = data.pose.position.y
	P[2] = data.pose.position.z
	P[3] = data.pose.covariance[0]
	ins_link.send_pos(P)
# imu_pub = rospy.Publisher("imu/cerberus", Imu, queue_size=10)
odom_pub = rospy.Publisher("odom/cerberus",Odometry,queue_size=10)
control_pub = rospy.Publisher("/car/mux/ackermann_cmd_mux/input/navigation", AckermannDriveStamped,queue_size =10)
vio_sub = rospy.Subscriber("/car/t265/odom/sample",Odometry, vel_BF_cb)
speed_variance = 1000
position_variance = 1000
fix_type = 1

def main():
	global ins_link
	ins_link.set_origin()
	r = rospy.Rate(100)
	while not rospy.is_shutdown():
		state = ins_link.state
		imu = ins_link.IMU
		control = ins_link.OPFlow # hack to pass control through the unused covariance variable :P
		now = rospy.Time.now()
		cur_odom = Odometry()
		cur_odom.header.frame_id = "/map"
		cur_odom.header.stamp = now
		# quaternion_multiply([sqrt(2)/2,sqrt(2)/2,0,0], quat_ned)
		cur_odom.pose.pose.position.x = state[8]
		cur_odom.pose.pose.position.y = state[7]
		cur_odom.pose.pose.position.z = -state[9]
		cur_odom.pose.pose.orientation.x = state[0]
		cur_odom.pose.pose.orientation.y = state[1]
		cur_odom.pose.pose.orientation.z = state[2]
		cur_odom.pose.pose.orientation.w = state[3]
		cur_odom.twist.twist.linear.x = state[5]
		cur_odom.twist.twist.linear.y = state[4]
		cur_odom.twist.twist.linear.z = -state[6]
		cur_odom.twist.twist.angular.x = imu[1]
		cur_odom.twist.twist.angular.y = imu[0]
		cur_odom.twist.twist.angular.z = -imu[2]
		odom_pub.publish(cur_odom)

		drive_msg = AckermannDriveStamped()
		drive_msg.header.frame_id=""
		drive_msg.drive.steering_angle = control[2]
		drive_msg.drive.speed = control[3]
		control_pub.publish(drive_msg)

		r.sleep()



if __name__ == '__main__':
	rospy.init_node('cerberus')
	main()