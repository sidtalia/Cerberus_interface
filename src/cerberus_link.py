#!/usr/bin/env python
import time
import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Point
from sensor_msgs.msg import Imu,NavSatFix
import math as m 
import traceback
from TRAC_link import *
import tf.transformations


ins_link = INS(COM='/dev/ttyUSB0',baud = 921600)

imu_quat = None
A = np.zeros(3)
G = np.zeros(3)
vel_time = time.time()
def vel_BF_cb(data):
	global ins_link
	global vel_time
	vx = data.twist.twist.linear.x
	vy = data.twist.twist.linear.y
	vz = data.twist.twist.linear.z
	# TODO: replace with something to convert vio from any orientation to any orientation.
    # right now it is hard coded to work with my setup in which
	th = 20/57.3 # the camera looks to the side and down at an angle of 20 degrees. Also it's inverted. Don't ask.
	V = np.zeros(4,dtype='float32')
	V[0] = -vy
	V[1] = vx*m.cos(th) - vz*m.sin(th)
	V[2] = vx*m.sin(th) + vz*m.cos(th)
	V[3] = data.twist.covariance[0]
	G = ins_link.IMU[3:6]
	BO = np.array([0.15,0.075,0.0]) #body frame offset	
	V[0] -= (G[1]*BO[2] - G[2]*BO[1])
	V[1] -= (G[2]*BO[0] + G[0]*BO[2])
	V[2] -= (G[1]*BO[0] + G[0]*BO[1])
	if(time.time() - vel_time>=0.05):
		vel_time = time.time()
		ins_link.send_vel_BF(V) # body frame xyz is front right down

odom_pub = rospy.Publisher("odom/cerberus",Odometry,queue_size=10)
navsat_pub = rospy.Publisher("gps/cerberus", NavSatFix, queue_size=10)
# TODO: parameterize:
control_pub = rospy.Publisher("/car/mux/ackermann_cmd_mux/input/navigation", AckermannDriveStamped,queue_size =10)
vio_sub = rospy.Subscriber("/car/t265/odom/sample",Odometry, vel_BF_cb)  # TODO: parameterize this
speed_variance = 1000  # starting variance for velocity
fix_type = 1  # GPS fix type

def main():
	global ins_link  # object that talks to the INS (cerberus in this case)
	ins_link.set_origin()  # send reset origin command to INS so that it resets the position to 0 when this script starts
	r = rospy.Rate(100)  # update rate
	cur_odom = Odometry()  # odometry object used for publishing..guess what..odometry!
	cur_odom.header.frame_id = "map"  # header frame id because we need one
	odom_broadcaster = tf.TransformBroadcaster()  # without this the orientation doesn't change in rviz.
	NSF = NavSatFix()  # for publishing GPS data in case some weirdo wants it
	NSF.header.frame_id = "map"
	while not rospy.is_shutdown():
		state = ins_link.state  # get the 13 element state: quaternion(4), vel(3), pos(3), AHRS rpy(3)
		imu = ins_link.IMU  # raw (but bias corrected and scaled) accel gyro mag data
		control = ins_link.OPFlow # OPFlow is an unused variable and was thus being used to pass controls through
		now = rospy.Time.now()
		cur_odom.header.stamp = now
		rpy = ins_link.quat2eul(state[0:4])
		odom_quat = tf.transformations.quaternion_from_euler(rpy[0],rpy[1],(m.pi/2)-rpy[2])
		cur_odom.pose.pose = Pose(Point(state[8], state[7], -state[9]), Quaternion(*odom_quat))
		cur_odom.twist.twist.linear.x = state[5]
		cur_odom.twist.twist.linear.y = state[4]
		cur_odom.twist.twist.linear.z = -state[6]
		cur_odom.twist.twist.angular.x = imu[4]
		cur_odom.twist.twist.angular.y = imu[3]
		cur_odom.twist.twist.angular.z = -imu[5]
		odom_pub.publish(cur_odom)
		odom_broadcaster.sendTransform((state[8], state[7], -state[9]),odom_quat,rospy.Time.now(),"base_link","odom")
		odom_broadcaster.sendTransform((0, 0, 0),(0,0,0,1),rospy.Time.now(),"odom","map")

		NSF.header.stamp = now
		NSF.latitude = ins_link.gps_data[0]
		NSF.longitude = ins_link.gps_data[1]
		NSF.altitude = ins_link.gps_data[2]
		NSF.position_covariance[0] = NSF.position_covariance[4] = ins_link.gps_data[6]
		NSF.position_covariance[8] = ins_link.gps_data[7]
		NSF.position_covariance_type = 3
		NSF.status.status = int(ins_link.ADS[0])

		navsat_pub.publish(NSF)

		drive_msg = AckermannDriveStamped()
		drive_msg.header.frame_id=""
		drive_msg.drive.steering_angle = 0.5*control[2]
		drive_msg.drive.speed = 3*control[3]
		control_pub.publish(drive_msg)

		r.sleep()



if __name__ == '__main__':
	rospy.init_node('cerberus')
	main()
