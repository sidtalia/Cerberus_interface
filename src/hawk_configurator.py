import yaml 
import numpy as np

class configuration():
    def __init__(self):
        self.conf = np.zeros(31)
        self.weight = np.ones(31)
        self.weight[:10] *= 4
        self.weight[10:] *= 2
        self.address = np.cumsum(self.weight)
        self.address -= 4
        self.weight = np.ones(31)
        self.weight[:9] *= 4
        self.weight[9:] *= 2

    def load_config(self, param_file):
        self.param_file = param_file
        config = None
        with open(param_file) as f:
            config = yaml.safe_load(f)
        self.conf[0] = config["Lt"]/(config["elevator_ratio"]*config["horizontal_stabilizer_ratio"]) # Lt/(elevator_ratio*stabilizer_ratio)
        self.conf[1] = config["elevator_ratio"] # ratio of elevator chord to horizontal stabilizer chord
        self.conf[2] = config["Lw"]/(config["aileron_ratio"]*config["aileron_wing_ratio"])
        self.conf[3] = config["aileron_ratio"]
        self.conf[4] = config["Lr"]/(config["rudder_ratio"]*config["vertical_stabilizer_ratio"])
        self.conf[5] = config["rudder_ratio"]
        self.speed_profile = np.array([0, config["1_speed"], config["2_speed"], config["3_speed"], config["4_speed"], config["5_speed"], config["6_speed"]])
        self.throttle_profile = np.array([0,float(1/6), float(1/3), 0.5, float(4/6), float(5/6), 1.0]) # this could have been done with a simple arange.
        self.conf[6:9] = np.polyfit(self.speed_profile,self.throttle_profile,2)
        self.conf[9] = config["elevator_max"]*100 # elevator deflection in centi degrees
        self.conf[10] = config["aileron_max"]*100 # aileron deflection (one side) in centi degrees
        self.conf[11] = config["rudder_max"]*100
        self.conf[12] = config["stall_speed"]*100 # stall speed in cm/s
        self.conf[13] = config["cruise_speed"]*100 # cruise speed at which the system will operate
        self.conf[14] = config["level_speed"]*100 # speed at which lift = weight of the plane with 0 elevator input
        self.conf[15] = config["land_speed"]*100 # speed at which plane lands
        self.conf[16]= config["flap_speed"]*100 # speed at which flaps are engaged
        self.conf[17] = 1000*config["cg_dist"] # distance between cog and col. +ve if cog is ahead of col (longitudenally stable)
        self.conf[18] = 1000*config["roll_Time_constant"]  # time constants in ms
        self.conf[19] = 1000*config["pitch_Time_constant"]
        self.conf[20] = 1000*config["yaw_Time_constant"]
        self.conf[21] = 1000*config["altitudeTc"] # FBWB parameter 
        self.conf[22] = 100*config["max_alt_rate"] # FBWB parameter 
        self.conf[23] = config["g_limit"] # acceleration limit (m/s/s) that the airframe should not exceed.
        self.conf[24] = 100*config["trajectory_time_constant"]
        self.conf[25] = 1000*config["rate_derivative_gain"] # multiply by 1e-3
        self.conf[26] = 1000*config["force_error_gain"] # multiply by 1e-3
        self.conf[27] = config["force_upper_limit"] # just convert to float
        self.conf[28] = config["rlim_pitch"] # rotation rate limit in radians/s
        self.conf[29] = config["rlim_roll"]
        self.conf[30] = config["rlim_yaw"]
        for i in range(len(self.conf)):
            print(self.conf[i], self.address[i], self.weight[i])
