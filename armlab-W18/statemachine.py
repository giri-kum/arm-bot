import datetime #Giri copy
import time
from kinematics import *
import numpy as np
import functools
import decimal
import pylab

D2R = 3.141592/180.0
R2D = 180.0/3.141592
current_state = "idle"
past_state = "idle"
goal_angles = [0.0]*4	
picking = "True"
def gen_timestamp(usec=False): #Giri
	t = datetime.datetime.now()
	if(usec):
		timestamp = str(t.day) + '_' + str(t.hour) + '_' + str(t.minute) + '_' + str(t.second) + '_' + str(t.microsecond)
	else:
		timestamp = str(t.day) + '_' + str(t.hour) + '_' + str(t.minute) + '_' + str(t.second)
	return timestamp


def isclose(x,y,tol):
	if(abs(x-y)<=tol):
		return True
	else:
		return False

def trim_angle(q):
	if(q>360 or q<-360):
		new_q = 0
	elif(q>180): #181 --> -179
		new_q = q-360
	elif(q<-180):
		new_q = 360-q
	else :
		new_q = q
	return new_q



class Statemachine():
    timerCallback = functools.partial(gen_timestamp)

    def setme(self, ui, rex):
	self.timerCallback = functools.partial(self.statemachine_check, ui = ui ,rex = rex)
   	
    def idle(self,rex):
	global current_state	
	current_state = "idle"
	#print "I am Idle"
	rex.speed_multiplier = [0.1]*rex.num_joints
        rex.cmd_publish()	
    def estop(self, rexarm):
        #sets the torque of all motors to 0, in case of emergency
        rexarm.max_torque = [0.0, 0.0, 0.0, 0.0]
        rexarm.cmd_publish()
        
        timerCallback = functools.partial(gen_timestamp)

    def checkmotors(self,rex):
	atol = 5.000 #do a formal way of selecting this tolerance
	"""
	print rex.joint_angles[0]*R2D-rex.joint_angles_fb[0]*R2D
	print rex.joint_angles[1]*R2D-rex.joint_angles_fb[1]*R2D
	print rex.joint_angles[2]*R2D-rex.joint_angles_fb[2]*R2D
	print rex.joint_angles[3]*R2D-rex.joint_angles_fb[3]*R2D
	"""	
	if (isclose(rex.joint_angles[0]*R2D,rex.joint_angles_fb[0]*R2D,atol)):
		if(isclose(rex.joint_angles[1]*R2D,rex.joint_angles_fb[1]*R2D,atol)):
			if(isclose(rex.joint_angles[2]*R2D,rex.joint_angles_fb[2]*R2D,atol)):
				if(isclose(rex.joint_angles[3]*R2D,rex.joint_angles_fb[3]*R2D,atol)):
					if(rex.num_joints==4):
						return True
					elif(rex.num_joints==6):
						if(isclose(rex.joint_angles[4]*R2D,rex.joint_angles_fb[4]*R2D,atol)):
							if(isclose(rex.joint_angles[5]*R2D,rex.joint_angles_fb[5]*R2D,atol)):
								return True
							else:
								#print "motor 5 fails" + str(abs(rex.joint_angles[5]*R2D-rex.joint_angles_fb[5]*R2D))
								return False
						else:
							#print "motor 4 fails" + str(abs(rex.joint_angles[4]*R2D-rex.joint_angles_fb[4]*R2D))
							return False
							
				else:
					#print "motor 3 fails" + str(abs(rex.joint_angles[3]*R2D-rex.joint_angles_fb[3]*R2D))
					return False
			else:
				#print "motor 2 fails" + str(abs(rex.joint_angles[2]*R2D-rex.joint_angles_fb[2]*R2D))
				return False
		else:
			#print "motor 1 fails" + str(abs(rex.joint_angles[1]*R2D-rex.joint_angles_fb[1]*R2D))
			return False
	else:
		#print "motor 0 fails " + str(abs(rex.joint_angles[0]*R2D-rex.joint_angles_fb[0]*R2D))
		return False 	

    
    def statemachine_check(self, ui, rex): #running at 100 ms
	global current_state, past_state
	
	if(current_state != past_state):
		print current_state
		past_state = current_state
	
	if(current_state == "holding"):
		if(self.checkmotors(rex)):
			self.reach(ui,rex)
	
	if(current_state == "picking" or current_state == "placing"):
		if(self.checkmotors(rex)):
			#print "reached " + current_state
			if(current_state=="picking"):
				self.close(rex)
			elif(current_state=="placing"):
				self.open(rex)
			else:
				pass#print "stupid"
		else:
			pass#print "not reached yet"
	elif(current_state == "opening" or current_state == "closing"):
		if(self.checkmotors(rex)):
			self.idle(rex)

    def reach(self,ui,rex):
	global current_state
	rex.joint_angles[0] = goal_angles[0]*D2R
        rex.joint_angles[1] = goal_angles[1]*D2R
        rex.joint_angles[2] = goal_angles[2]*D2R
        rex.joint_angles[3] = goal_angles[3]*D2R	 
        if(picking):
		current_state = "picking"
	else:
		current_state = "placing"
    	rex.cmd_publish()
	
    def close(self,rex):
	global current_state
	#print "Hello from pick"
	rex.joint_angles[5] = -90*D2R
	rex.cmd_publish()
	current_state = "closing"    		

    def open(self,rex):
	global current_state
	rex.joint_angles[5] = 90*D2R
	rex.cmd_publish()
	current_state = "opening"

    def hold(self,ui,rex,angles,state):
	global current_state,goal_angles, picking
	goal_angles[0] = angles[0]
	goal_angles[1] = angles[1]
	goal_angles[2] = angles[2]
	goal_angles[3] = angles[3]
	if(state == "picking"):
		picking = True
		rex.joint_angles[5] = 90*D2R
	else:
		rex.joint_angles[5] = -90*D2R
		picking = False
	rex.joint_angles[0] = 0.0
        rex.joint_angles[1] = 0.0
        rex.joint_angles[2] = 0.0
        rex.joint_angles[3] = 0.0
     	rex.joint_angles[4] = 0.0
		
	rex.cmd_publish()
	current_state = "holding"
	
	    		
