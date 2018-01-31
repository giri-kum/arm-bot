import lcm
import time
import numpy as np
import sys
sys.path.append("lcmtypes")

from lcmtypes import dynamixel_command_t
from lcmtypes import dynamixel_command_list_t
from lcmtypes import dynamixel_status_t
from lcmtypes import dynamixel_status_list_t
from lcmtypes import dynamixel_config_t
from lcmtypes import dynamixel_config_list_t

"""Useful Conversions"""
PI = np.pi
D2R = PI/180.0
ANGLE_TOL = 2*PI/180.0 


""" Rexarm Class """
class Rexarm():
    def __init__(self):

        """ TODO: modify this class to add functionality you need """

        """ Commanded Values """
        self.num_joints = 6                         #Giri #number of motors, increase when adding gripper
        self.joint_angles = [0.0] * self.num_joints # radians
        
        # you must change this to an array to control each joint speed separately 
        self.speed = 1.0        #Giri 
        self.max_torque = [1.0] * self.num_joints     #Giri, 1.0, 1.0, 1.0]                    # 0 to 1
        self.speed_multiplier = [0.5]* self.num_joints
        self.torque_multiplier = 0.5

        """ Feedback Values """
        self.joint_angles_fb = [0.0] * self.num_joints # radians
        self.speed_fb = [0.0] * self.num_joints        # 0 to 1   
        self.load_fb = [0.0] * self.num_joints         # -1 to 1  
        self.temp_fb = [0.0] * self.num_joints         # Celsius               

        """ Waypoint Plan - TO BE USED LATER """
        self.plan = []
        self.plan_status = 0
        self.wpt_number = 0
        self.wpt_total = 0

        """ Setup LCM and subscribe """
        self.lc = lcm.LCM()
        lcmMotorSub = self.lc.subscribe("DXL_STATUS",
                                        self.feedback_handler)

    def cmd_publish(self):
        """ 
        Publish the commands to the arm using LCM.
        This publishes the joint angles currently set in the Rexarm object
        You need to call this function to command the arm.
        """    
        msg = dynamixel_command_list_t()
        msg.len = self.num_joints
        self.clamp()
        
        for i in range(msg.len):
            cmd = dynamixel_command_t()
            cmd.utime = int(time.time() * 1e6)
            cmd.position_radians = self.joint_angles[i]
	    #print self.speed_multiplier
            cmd.speed = self.speed_multiplier[i] * self.speed
            cmd.max_torque = self.torque_multiplier * self.max_torque[i]
            msg.commands.append(cmd)
        self.lc.publish("DXL_COMMAND",msg.encode())

    def cfg_publish_default(self):
        """ 
        Publish default configuration to arm using LCM. 
        """    
        msg = dynamixel_config_list_t()
        msg.len = self.num_joints
        for i in range(msg.len):
            cfg = dynamixel_config_t()
            cfg.utime = int(time.time() * 1e6)
            cfg.kp = 32
            cfg.ki = 32
            cfg.kd = 0
            cfg.compl_margin = 0
            cfg.compl_slope = 16
            cfg.LED = 1
            msg.configs.append(cfg)
        self.lc.publish("DXL_CONFIG",msg.encode())

    def cfg_publish(self):
        """ 
        TODO: implement this function (optional)

        Publish configuration to arm using LCM. 
        You need to activelly call this function to command the arm.
        """    
        pass
    
    def get_feedback(self):
        """
        LCM Handler function
        Called continuously with timer by control_station.py
        times out after 10ms
        """
        self.lc.handle_timeout(10)

    def feedback_handler(self, channel, data):
        """
        Feedback Handler for LCM
        this is run when a feedback message is recieved
        """
        msg = dynamixel_status_list_t.decode(data)
        for i in range(msg.len):
            self.joint_angles_fb[i] = msg.statuses[i].position_radians 
            self.speed_fb[i] = msg.statuses[i].speed 
            self.load_fb[i] = msg.statuses[i].load 
            self.temp_fb[i] = msg.statuses[i].temperature

    def clamp(self): # Giri
        """
        TODO: implement this function

        Clamp Function
        Limit the commanded joint angles to ones physically possible 
        so the arm is not damaged.
	joint_angles[0] - Base, [1] - Shoulder , [2] -Arm, [3] - Wrist
	"""
        pass
	"""
	low_limit = [-175, -120, -120, -100]#[-175, -120, -120, -120] #[-179, -122.59, -122.59, -128.61]
	up_limit = [175, 125, 120, 100]#[175, 125, 120, 125]#[179, 128.04, 121.36, 128.61]
	for i in range(0,4):
		if self.joint_angles[i] > up_limit[i]*D2R:
			print "upper clamp motor " + str(i) + "Can't go to: " + str(self.joint_angles[i])
			self.joint_angles[i] = up_limit[i]*D2R
		elif self.joint_angles[i] < low_limit[i]*D2R:
			print "lower clamp motor " + str(i) + "Can't go to: " + str(self.joint_angles[i])
			self.joint_angles[i] = low_limit[i]*D2R
	"""		
	 
    def rexarm_FK(dh_table, link):
        """
        TODO: implement this function

        Calculates forward kinematics for rexarm
        takes a DH table filled with DH parameters of the arm
        and the link to return the position for
        returns a 4-tuple (x, y, z, phi) representing the pose of the 
        desired link
        """
        pass
    	
    def rexarm_IK(pose, cfg):
        """
        TODO: implement this function

        Calculates inverse kinematics for the rexarm
        pose is a tuple (x, y, z, phi) which describes the desired
        end effector position and orientation.  
        cfg describe elbow down (0) or elbow up (1) configuration
        returns a 4-tuple of joint angles or NONE if configuration is impossible
        """
        pass
        
    def rexarm_collision_check(q):
        """
        TODO: implement this function

        Perform a collision check with the ground and the base of the Rexarm
        takes a 4-tuple of joint angles q
        returns true if no collision occurs
        """
        pass 
