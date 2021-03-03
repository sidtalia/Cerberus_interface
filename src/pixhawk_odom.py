#!/usr/bin/env python
import time
import sys, os
import numpy as np
#source : https://gist.github.com/vo/9331349
import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TwistStamped, Vector3
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pymavlink import mavutil
import math as m 
import traceback

imu_quat = None
init_pos = np.zeros(3)
A = np.zeros(3)
G = np.zeros(3)
imu_pub = rospy.Publisher("imu/pixhawk", Imu, queue_size=10)
odom_pub = rospy.Publisher("odom/pixhawk",Odometry,queue_size=10)
speed_variance = 1000
position_variance = 1000
fix_type = 1

def calcposNED(lat, lon, hgt, latReference, lonReference, hgtReference):
	earthRadius = 6378145.0
	lat /= 57.3
	lon /= 57.3
	latReference /= 57.3
	lonReference /= 57.3
	posNEDr = np.zeros(3)
	posNEDr[0] = earthRadius * (lat - latReference)
	posNEDr[1] = earthRadius * m.cos(latReference) * (lon - lonReference)
	posNEDr[2] = -(hgt - hgtReference)
	return posNEDr


def handle_heartbeat(msg):
	mode = mavutil.mode_string_v10(msg)
	is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
	is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED

def handle_rc_raw(msg):
	channels = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, 
			msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw)

def handle_hud(msg):
	hud_data = (msg.airspeed, msg.groundspeed, msg.heading, 
				msg.throttle, msg.alt, msg.climb)
	# print "Aspd\tGspd\tHead\tThro\tAlt\tClimb"
	# print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f" % hud_data

def handle_attitude(msg):
	global imu_quat
	global G
	global A
	global imu_pub
	attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed, 
				msg.pitchspeed, msg.yawspeed)
	yaw = -msg.yaw
	yaw += m.pi/2
	if(yaw>m.pi):
		yaw -= 2*m.pi
	if(yaw< -m.pi):
		yaw += 2*m.pi

	G = np.array([msg.rollspeed,msg.pitchspeed,-msg.yawspeed])
	A[0] -= 9.89*m.sin(msg.pitch)
	if(m.fabs(A[0])<0.1):
		A[0] = 0
	A[0] = round(A[0],2)
	A[1] += 9.89*m.sin(msg.roll)
	if(m.fabs(A[1])<0.1):
		A[1] = 0
	A[1] = round(A[1],2)

	imu = Imu()
	imu.header.stamp = rospy.Time.now()
	imu.header.frame_id = "base_link"
	imu_quat = quaternion_from_euler(msg.roll, msg.pitch, yaw)
	imu.orientation = Quaternion(*imu_quat)
	imu.linear_acceleration.x = -A[0]
	imu.linear_acceleration.y = -A[1]
	imu.linear_acceleration.z = -A[2]
	imu.angular_velocity.x = G[0]
	imu.angular_velocity.y = G[1]
	imu.angular_velocity.z = G[2]
	imu_pub.publish(imu)
	return


def handle_acc(msg):
	global A
	A = np.array([float(msg.xacc)*0.01,float(msg.yacc)*0.01,float(msg.zacc)*0.01])

def handle_position(msg):
	global speed_variance
	global position_variance
	global twist_pub
	global G
	global fix_type
	global init_pos
	if(fix_type<3):
		return
	else:
		if(np.all(init_pos)==0):
			init_pos[0] = msg.lat*1e-7
			init_pos[1] = msg.lon*1e-7
			init_pos[2] = msg.alt*1e-3
			return
		now = rospy.Time.now()

		cur_odom = Odometry()
		cur_odom.header.frame_id = "map"
		cur_odom.header.stamp = now

		posNED = calcposNED(msg.lat*1e-7,msg.lon*1e-7,msg.alt*1e-3,init_pos[0],init_pos[1], init_pos[2])
		cur_odom.pose.pose.position.x = posNED[1] # convert from NED to ENU
		cur_odom.pose.pose.position.y = posNED[0]
		cur_odom.pose.pose.position.z = -posNED[2]
		if imu_quat is not None:
			cur_odom.pose.pose.orientation = Quaternion(*imu_quat)
		cur_odom.twist.twist.linear.x = msg.vy
		cur_odom.twist.twist.linear.y = msg.vx
		cur_odom.twist.twist.linear.z = -msg.vz
		cur_odom.twist.twist.angular.x = G[1]
		cur_odom.twist.twist.angular.y = G[0]
		cur_odom.twist.twist.angular.z = -G[2]

		odom_pub.publish(cur_odom)



#new stuff
def handle_report(msg):
	global position_variance
	global speed_variance
	global fix_type
	if(fix_type>3):
		speed_variance = msg.velocity_variance
		position_variance = msg.pos_horiz_variance
	else:
		speed_variance = 1e7
		position_variance = 1e7

def handle_gps_type(msg):
	global fix_type
	fix_type = msg.fix_type
#new stuff

def read_loop(m):
	now = time.time()

	while not rospy.is_shutdown():
		# grab a mavlink message
		msg = m.recv_match(blocking=False)
		try:
			# handle the message based on its type
			if(msg is not None):
				msg_type = msg.get_type()
			else:
				msg_type = None
			if msg_type == "BAD_DATA":
				if mavutil.all_printable(msg.data):
					sys.stdout.write(msg.data)
					sys.stdout.flush()
			elif msg_type == "RC_CHANNELS_RAW": 
				handle_rc_raw(msg)
			elif msg_type == "HEARTBEAT":
				handle_heartbeat(msg)
			elif msg_type == "ATTITUDE":
				handle_attitude(msg)
			elif msg_type == "GLOBAL_POSITION_INT":
				handle_position(msg)
			elif msg_type == "EKF_STATUS_REPORT":
				handle_report(msg)
			elif msg_type == 'RAW_IMU':
				handle_acc(msg)
			elif msg_type == 'GPS_RAW_INT':
				handle_gps_type(msg)

		except KeyboardInterrupt:
			break
		except Exception as e:
			print(traceback.format_exc())
		except:
			pass
		
def main():
	rate = 50
	device = '/dev/ttyACM0'
	baudrate = 115200


	# create a mavlink serial instance
	try:
		master = mavutil.mavlink_connection(device, baud=baudrate)
	except:
		master = mavutil.mavlink_connection('/dev/ttyACM1', baud=baudrate)


	# wait for the heartbeat msg to find the system ID
	master.wait_heartbeat()

	# request data to be sent at the given rate
	master.mav.request_data_stream_send(master.target_system, master.target_component, 
		mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)
	# enter the data loop
	read_loop(master)


if __name__ == '__main__':
	rospy.init_node('pixhawk')
	main()
