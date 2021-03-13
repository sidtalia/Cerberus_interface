import DAVID_COMS 
from DAVID_COMS import *
import time
import traceback
import math as m
import threading

class INS(object):
	def __init__(self, COM='/dev/ttyUSB0',baud = 921600):
		self.BAUD = baud
		self.com = com_handler()
		self.OFFSET_ID = 0x0001
		self.COMMAND_ID = 0X0002
		self.START_ID = 0x00FE
		self.WP_ID = 0x0005
		self.STATE_ID = 0x0006
		self.SENSOR_ID = 0x0016
		self.CALIB_ID = 0x000A
		self.CLEAR_ID = 0x0008
		self.SET_ORIGIN_ID = 0x000B
		self.VEL_ID = 0x0017
		self.POS_ID = 0x0018
		self.REC_ID_1 = 0x000C
		self.REC_ID_0 = 0x00FC
		self.REC_DEBUG_ID_1 = 0x000D
		self.REC_DEBUG_ID_0 = 0x00FD
		self.GYRO_CAL = 0x10
		self.ACCEL_CAL = 0x20
		self.MAG_CAL = 0x30
		self.ERROR_CODE = 0xFF
		self.Tx_MODE = 0x01 #default starting mode 
		self.Tx_ID = self.STATE_ID #default message ID 
		self.Tx_msg_len = 28 #default message length
		self.rec = False
		self.rec_vid = False
		self.point_count = 0
		self.waypoint_list = []
		self.waypoint_length = 0
		self.waypoint_file = None
		self.offset_file_name = 'ins_offsets.npy'
		self.received_wp_list = []

		self.MODE = 0x01
		self.saved = False

		self.state = np.zeros(13)
		self.Covariance = np.zeros(4)
		self.max_exec_time = 0
		self.ins_rpy = np.zeros(3)
		self.gps_data = np.zeros(9)
		self.ADS = np.zeros(2)
		self.IMU = np.zeros(9)
		self.OPFlow = np.zeros(4)
		self.UID = 0x0000
		self.sleep_time = 0.001
		self.data_in = np.zeros(4)

		if(not self.connect(self.BAUD,COM)):
			print("error: com device not connected")
			exit()

		self.read_proc = threading.Thread(target=self.loop) # daemon -> kills thread when main program is stopped
		self.read_proc.setDaemon(True)
		self.read_proc.start()
		self.send_timer = time.time()

	def connect(self,BAUD,COM='/dev/ttyUSB0'):
		try:
			self.com._init_(COM,BAUD,68)#last parameter is max bytes
			return True
		except Exception as e:
			print(traceback.format_exc())
		except:
			return False

	def quat2eul(self,u):
		y = np.zeros(3)
		y[0] = m.atan2((2.0*(u[2]*u[3]+u[0]*u[1])) , (u[0]*u[0]-u[1]*u[1]-u[2]*u[2]+u[3]*u[3]));
		y[1] = -m.asin(2.0*(u[1]*u[3]-u[0]*u[2]));
		y[2] = m.atan2((2.0*(u[1]*u[2]+u[0]*u[3])) , (u[0]*u[0]+u[1]*u[1]-u[2]*u[2]-u[3]*u[3]));
		return y

	def set_standby(self):
		self.Tx_MODE = 0x01

	def send_vel_BF(self,V):
		self.data_in = V
		self.Tx_ID = self.VEL_ID
	def send_pos(self,P):
		self.data_in = P
		self.Tx_ID = self.POS_ID

	def send_heartbeat(self):
		message_id = np.array([self.START_ID,self.Tx_msg_len,self.Tx_ID,self.Tx_MODE],dtype = 'int16') 
		if(self.Tx_ID == self.WP_ID or self.point_count>0):
			message_id = np.array([self.START_ID,self.waypoint_length,self.WP_ID,self.Tx_MODE],dtype = 'int16') 
			n = self.waypoint_length - self.point_count
			X = int(self.waypoint_list[n][0]*1e2)
			Y = int(self.waypoint_list[n][1]*1e2)
			height = int(self.waypoint_list[n][2]*1e2)
			message = list(np.array([X,Y,height,n],dtype='int16'))
			print(message)
			message_id = np.concatenate((message_id,message),axis=0)
		if(self.point_count<0 and self.Tx_ID == self.WP_ID):
			self.Tx_ID = self.STATE_ID
		if(self.Tx_ID == self.VEL_ID or self.Tx_ID == self.POS_ID):
			X = self.data_in.tobytes()
			X = np.frombuffer(X,dtype='int16')
			message_id = np.concatenate((message_id,X),axis=0)
			X = X.tobytes()
			X = np.frombuffer(X,dtype='float32')
			print(X)
		self.com.send(message_id)
		if(self.Tx_MODE == 0x07):
			self.set_standby()

	def handle_car_status(self,status):
		if (status & 0x0002 == 1) and (self.rec_vid==False):
			self.Tx_ID = self.REC_ID_0
		elif (status & 0x0002==0) and (self.rec_vid==True):
			self.Tx_ID = self.REC_ID_1
		if (status&0x0001==1) and (self.rec==False):
			self.Tx_ID = self.REC_DEBUG_ID_0
		elif (status&0x0001==0) and (self.rec==True):
			self.Tx_ID = self.REC_DEBUG_ID_1

	def print_data(self):
		print(self.state, self.max_exec_time)

	def get_Quat_Vel_Pose(self):
		Quat = self.state[:4]
		Vel = self.state[4:7]
		Pos = self.state[7:10]
		return Quat,Vel,Pos

	def loop(self):
		while True:
			self.readSerial()
			time.sleep(self.sleep_time)

	def set_origin(self):
		self.Tx_ID = self.SET_ORIGIN_ID

	def calib(self):
		self.saved = False
		self.Tx_ID = self.CALIB_ID

	def record(self):
		self.rec = True
		self.Tx_ID = self.REC_DEBUG_ID_1

	def stop_recording(self):
		self.rec = False
		self.Tx_ID = self.REC_DEBUG_ID_0

	def readSerial(self):
		try:
			num_bytes = self.com.check_recv()
			if(num_bytes>=4):
				while(num_bytes<68):
					# time.sleep(0.00001)
					num_bytes = self.com.check_recv()
				message = self.com.read(68)

				START_SIGN = message[0]
				ID = message[1]

				if ID == self.STATE_ID or ID == self.SENSOR_ID:

					if(ID == self.STATE_ID):
						print('state message',num_bytes) # debug
						self.state = np.frombuffer(message[2:28],dtype = np.float32)
						self.Covariance = np.frombuffer(message[28:32], dtype='uint16')
						self.max_exec_time = np.frombuffer(message[32], dtype='uint16')
						self.UID = np.frombuffer(message[33:34], dtype='uint16')
						self.ins_rpy = self.quat2eul(self.state[0:4])

					if(ID == self.SENSOR_ID):
						# print('sensor_message',num_bytes)
						buflong = np.frombuffer(message[2:12],dtype=np.int32)
						bufint = message[12:28]
						bufint8 = np.frombuffer(message[28:30],dtype=np.int8)
						self.gps_data[0] = float(buflong[0])*1e-7
						self.gps_data[1] = float(buflong[1])*1e-7
						self.gps_data[2] = float(buflong[2])*1e-3
						self.gps_data[3:6] = np.array(bufint[4:7],dtype=np.float32)*1e-2
						self.gps_data[6:9] = np.array(bufint[:3],dtype=np.float32)*1e-2

						self.ADS[0] = float(bufint[3])*1e-2 # airspeed
						self.ADS[1] = float(buflong[3])*1e-3

						for i in range(3):
							self.IMU[i] = float(bufint[7+i])*1e-2
							self.IMU[i+3] = float(bufint[10+i])*1e-2
							self.IMU[i+6] = float(bufint[13+i])*1e-2
						for i in range(4):
							self.OPFlow[i] = float(bufint8[i])*0.01

					if(time.time() - self.send_timer>=0.02): # regulate heartbeat at 100 hz
						self.send_heartbeat()
						self.Tx_ID = self.STATE_ID #reset to state ID. I don't want it to continuously register waypoints

				if ID == self.ACCEL_CAL:
					num = message[2]
					if(num == 0):
						print("put vehicle on the level")
					if(num == 1):
						print("put vehicle on the left side")
					if(num == 2):
						print("put vehicle on the right side")
					if(num == 3):
						print("put the vehicle on front side")
					if(num == 4):
						print("put the vehicle on it's rear side")
					if(num == 5):
						print("put the vehicle on it's back side")
					if(num == 6):
						print("start rotating the INS")
					if(num == 7):
						print("caliberation is done. please restart")

				if ID == self.ERROR_CODE:
					print("Maximum execution time exceeded. If this is a recurring problem, Please inform the manufacturer")

		except KeyboardInterrupt:
			self.com.close()
			exit()
		
		except Exception as e:
			print(traceback.format_exc())

		except:
			pass

if __name__ == '__main__':
	ins_link = INS(COM='/dev/ttyUSB0')
	while True:
		try:
			print(ins_link.get_Quat_Vel_Pose()) # note that quaternion, velocity and position are in the NED reference frame
			time.sleep(0.5)
		except KeyboardInterrupt:
			exit()
		except Exception as e:
			print(traceback.format_exc())
		except:
			pass