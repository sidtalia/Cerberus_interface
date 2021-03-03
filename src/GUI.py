from DAVID_link import *
import time
import os
import traceback
import matplotlib.pyplot as plt
plt.ion()
fig = plt.figure()
plt.axis('equal')
import tkinter as tk
from tkinter import *

log_file_name = 'INS_log_0.npy'
log_file = []

ins_link = INS(COM='/dev/ttyUSB0')

def update_display():
	try:
		global ins_link
		global log_file
		global log_file_name

		state = ins_link.state
		Covariance = ins_link.Covariance
		ins_rpy = ins_link.ins_rpy
		max_exec_time = ins_link.max_exec_time
		gps_data = ins_link.gps_data
		IMU = ins_link.IMU
		ADS = ins_link.ADS
		OPFlow = ins_link.OPFlow

		gcs.posX.configure(text = 'filtered X = {} meters'.format(str(round(state[8],2) ) ) )
		gcs.posY.configure(text = 'filtered Y = {} meters'.format(str(round(state[7],2) ) ) )
		gcs.posZ.configure(text = 'filtered height = {} meters'.format(str(round(-state[9],2) ) ) )
		gcs.velX.configure(text = 'VX = {} m/s'.format(str(round(state[5],2) ) ) )				
		gcs.velY.configure(text = 'VY = {} m/s'.format(str(round(state[4],2) ) ) )
		gcs.velZ.configure(text = 'VZ = {} m/s'.format(str(round(-state[6],2) ) ) )
		gcs.insRoll.configure(text = 'ins roll = {} degrees'.format(str( round(57.3*ins_rpy[0],2) ) ) )
		gcs.insPitch.configure(text = 'ins pitch = {} degrees'.format(str(round(57.3*ins_rpy[1],2) ) ) )
		gcs.insHeading.configure(text = 'ins heading = {} degrees'.format( str(round(57.3*ins_rpy[2],2) ) ) )

		gcs.imuRoll.configure(text = 'imu roll = {} degrees'.format(str( round(57.3*state[10],2) ) ) )
		gcs.imuPitch.configure(text = 'imu pitch = {} degrees'.format(str(round(57.3*state[11],2) ) ) )
		gcs.imuHeading.configure(text = 'imu heading = {} degrees'.format( str(round(57.3*state[12],2) ) ) )

		gcs.pos_cov.configure(text = 'pos. cov. = {} m'.format( str( round(Covariance[0],3)) ))
		gcs.vert_cov.configure(text = 'vert. cov. = {} m'.format( str( round(Covariance[1],3) ) ) )
		gcs.vel_cov.configure(text = 'vel. cov. = {} m/s'.format( str( round(Covariance[2],3) ) ) )
		gcs.vert_vc.configure(text = 'vert vel. cov. = {} m/s'.format( str( round( Covariance[3], 3) ) ) )
		gcs.Exec_time.configure(text = 'max_exec_time = {} microseconds'.format(str( round(max_exec_time,2) ) ) )

		gcs.gps_lon.configure(text = 'gps lon = {} degrees'.format( str( round(gps_data[0],7)) ))
		gcs.gps_lat.configure(text = 'gps lat = {} degrees'.format( str( round(gps_data[1],7)) ))
		gcs.gps_alt.configure(text = 'gps alt = {} m'.format( str( round(gps_data[2],2)) ))
		gcs.gps_velN.configure(text = 'gps VelN = {} m/s'.format( str( round(gps_data[3],2)) ))
		gcs.gps_velE.configure(text = 'gps VelE = {} m/s'.format( str( round(gps_data[4],2)) ))
		gcs.gps_velD.configure(text = 'gps VelD = {} m/s'.format( str( round(gps_data[5],2)) ))
		gcs.gps_hAcc.configure(text = 'gps hAcc = {} m'.format( str( round(gps_data[6],2)) ))
		gcs.gps_vAcc.configure(text = 'gps vAcc = {} m'.format( str( round(gps_data[7],2)) ))
		gcs.gps_sAcc.configure(text = 'gps sAcc = {} m/s'.format( str( round(gps_data[8],2)) ))
		gcs.Accel_X.configure(text = 'Ax = {} m/s2'.format( str( round(IMU[0], 2)) ))
		gcs.Accel_Y.configure(text = 'Ay = {} m/s2'.format( str( round(IMU[1], 2)) ))
		gcs.Accel_Z.configure(text = 'Az = {} m/s2'.format( str( round(IMU[2], 2)) ))
		gcs.Gyro_X.configure(text = 'Gx = {} rad/s'.format( str( round(IMU[3], 2)) ))
		gcs.Gyro_Y.configure(text = 'Gy = {} rad/s'.format( str( round(IMU[4], 2)) ))
		gcs.Gyro_Z.configure(text = 'Gz = {} rad/s'.format( str( round(IMU[5], 2)) ))
		gcs.Mag_X.configure(text = 'Mx = {} guass'.format( str( round(IMU[6], 2)) ))
		gcs.Mag_Y.configure(text = 'My = {} guass'.format( str( round(IMU[7], 2)) ))
		gcs.Mag_Z.configure(text = 'Mz = {} guass'.format( str( round(IMU[8], 2)) ))
		gcs.airspeed.configure(text = 'airspeed = {} m/s'.format( str( round(ADS[0], 2)) ))
		gcs.baro_alt.configure(text = 'baro alt = {} m'.format( str( round(ADS[1], 2)) ))
		gcs.opticalFlow_X.configure(text = 'OPFlow X = {}'.format( str( round(OPFlow[0], 2)) ))
		gcs.opticalFlow_Y.configure(text = 'OPFlow Y = {}'.format( str( round(OPFlow[1], 2)) ))
		gcs.opticalFlow_SQ.configure(text = 'OPFlow SQ = {}'.format( str( round(OPFlow[2], 2)) ))
		gcs.opticalFlow_use.configure(text = 'OPFlow use = {}'.format( str( round(OPFlow[3], 2)) ))
		if(ins_link.rec):
			data = np.array([state[0],state[1],state[2],state[3],state[4],state[5],state[6],state[7],state[8],state[9],state[10],state[11],state[12],
							Covariance[0],Covariance[1],Covariance[2],Covariance[3],max_exec_time,
							gps_data[0],gps_data[1],gps_data[2],gps_data[3],gps_data[4],gps_data[5],gps_data[6],gps_data[7],gps_data[8],
							ADS[0],ADS[1],
							IMU[0],IMU[1],IMU[2],IMU[3],IMU[4],IMU[5],IMU[6],IMU[7],IMU[8],
							OPFlow[0],OPFlow[1],OPFlow[2],OPFlow[3]])
			log_file.append(data)
			plt.scatter(state[8],state[7])
			plt.axis('equal')
			plt.show()
		else:
			if(len(log_file)):
				a = time.localtime(time.time())
				log_file_name = 'INS_log_{}_{}_{}_{}_{}.npy'.format(a.tm_year,a.tm_mon,a.tm_mday,a.tm_hour,a.tm_min)
				np.save( log_file_name, log_file)
				log_file = []#reset
				plt.clf() # clear the points
	except Exception as e:
		print(traceback.format_exc())
	except:
		pass
	gcs.root.after(10, update_display)


class GCS():
	def __init__(self):
		DISPLAY_WIDTH  = 400
		DISPLAY_HEIGHT = 800
		BACKGROUND_COLOR = 'white'
		global ins_link
		self.root = tk.Tk()
		self.root.configure(bg=BACKGROUND_COLOR)
		self.root.resizable(False, False)
		self.root.title('DAVID Ground Control Station')
		left = (self.root.winfo_screenwidth() - DISPLAY_WIDTH) / 2
		top = (self.root.winfo_screenheight() - DISPLAY_HEIGHT) / 2
		self.root.geometry('%dx%d+%d+%d' % (DISPLAY_WIDTH, DISPLAY_HEIGHT, left, top))
		self.frame = tk.Frame(self.root)
		self.frame.pack(side=tk.LEFT)
		self.root.after(10, update_display)

		self.posX = tk.Label(self.frame,text='')
		self.posX.pack()
		self.posY = tk.Label(self.frame,text='')
		self.posY.pack()
		self.posZ = tk.Label(self.frame,text='')
		self.posZ.pack()
		self.velX = tk.Label(self.frame,text='')
		self.velX.pack()
		self.velY = tk.Label(self.frame,text='')
		self.velY.pack()
		self.velZ = tk.Label(self.frame,text='')
		self.velZ.pack()
		self.insRoll = tk.Label(self.frame,text='')
		self.insRoll.pack()
		self.insPitch = tk.Label(self.frame,text='')
		self.insPitch.pack()
		self.insHeading = tk.Label(self.frame,text='')
		self.insHeading.pack()
		self.imuRoll = tk.Label(self.frame,text='')
		self.imuRoll.pack()
		self.imuPitch = tk.Label(self.frame,text='')
		self.imuPitch.pack()
		self.imuHeading = tk.Label(self.frame,text='')
		self.imuHeading.pack()
		self.pos_cov = tk.Label(self.frame,text='')
		self.pos_cov.pack()
		self.vert_cov = tk.Label(self.frame,text='')
		self.vert_cov.pack()
		self.vel_cov = tk.Label(self.frame,text='')
		self.vel_cov.pack()
		self.vert_vc = tk.Label(self.frame,text='')
		self.vert_vc.pack()
		self.Exec_time = tk.Label(self.frame, text='')
		self.Exec_time.pack()

		self.gps_lon = tk.Label(self.frame, text='')
		self.gps_lon.pack()
		self.gps_lat = tk.Label(self.frame, text='')
		self.gps_lat.pack()
		self.gps_alt = tk.Label(self.frame, text='')
		self.gps_alt.pack()
		self.gps_velN = tk.Label(self.frame, text='')
		self.gps_velN.pack()
		self.gps_velE = tk.Label(self.frame, text='')
		self.gps_velE.pack()
		self.gps_velD = tk.Label(self.frame, text='')
		self.gps_velD.pack()
		self.gps_hAcc = tk.Label(self.frame, text='')
		self.gps_hAcc.pack()
		self.gps_vAcc = tk.Label(self.frame, text='')
		self.gps_vAcc.pack()
		self.gps_sAcc = tk.Label(self.frame, text='')
		self.gps_sAcc.pack()

		self.Accel_X = tk.Label(self.frame, text='')
		self.Accel_X.pack()
		self.Accel_Y = tk.Label(self.frame, text='')
		self.Accel_Y.pack()
		self.Accel_Z = tk.Label(self.frame, text='')
		self.Accel_Z.pack()
		self.Gyro_X = tk.Label(self.frame, text='')
		self.Gyro_X.pack()
		self.Gyro_Y = tk.Label(self.frame, text='')
		self.Gyro_Y.pack()
		self.Gyro_Z = tk.Label(self.frame, text='')
		self.Gyro_Z.pack()
		self.Mag_X = tk.Label(self.frame, text='')
		self.Mag_X.pack()
		self.Mag_Y = tk.Label(self.frame, text='')
		self.Mag_Y.pack()
		self.Mag_Z = tk.Label(self.frame, text='')
		self.Mag_Z.pack()

		self.airspeed = tk.Label(self.frame, text='')
		self.airspeed.pack()
		self.baro_alt = tk.Label(self.frame, text='')
		self.baro_alt.pack()

		self.opticalFlow_X = tk.Label(self.frame, text='')
		self.opticalFlow_X.pack()
		self.opticalFlow_Y = tk.Label(self.frame, text='')
		self.opticalFlow_Y.pack()
		self.opticalFlow_SQ = tk.Label(self.frame, text='')
		self.opticalFlow_SQ.pack()
		self.opticalFlow_use = tk.Label(self.frame, text='')
		self.opticalFlow_use.pack()


		self.button_frame = tk.Frame(self.root)
		self.button_frame.pack(side=tk.LEFT)

		self.mark_button = tk.Button(self.button_frame, text = 'reset system origin', command = ins_link.set_origin)
		self.mark_button.pack()
		self.record_button = tk.Button(self.button_frame, text = 'Record data', command = ins_link.record)
		self.record_button.pack()
		self.record_stop_button = tk.Button(self.button_frame, text = 'Stop recording', command = ins_link.stop_recording)
		self.record_stop_button.pack()
		self.caliberate_button = tk.Button(self.button_frame, text = 'Caliberation', command = ins_link.calib)
		self.caliberate_button.pack()

gcs = GCS()
tk.mainloop()


