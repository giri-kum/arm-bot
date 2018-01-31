import datetime #Giri copy
import time
from kinematics import *
import numpy as np
import functools
import decimal
import pylab

D2R = 3.141592/180.0
R2D = 180.0/3.141592
current_motorstate = "idle" # possible states: motion or idle
current_mode = "idle" # competition1 or competition2 or competition3 or competition4 or competition5 or testing
current_action = "idle" #picking, placing
current_movement = "idle" #picking, grabbing qi, grabbing q, grabbing qf, idle or placing, placing qi, placing q, placing qf  

comp1_status = "idle" # idle, red, blue, green
comp2_status = -1 # 0,1,2,
past_status = ""
past_states = [""]*4
qi = [0.0]*6
qf = [0.0]*6
q  = [0.0]*6
#picking = "True"
#mode ="normal"
wrist_orientation = 0.0
gripper_orientation = 90.0
q_comp2 = np.zeros([4,3])
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
    def setorientation(self, w_orient, g_orient=gripper_orientation,gripper=False):
	global wrist_orientation, gripper_orientation
	wrist_orientation = w_orient
	if(gripper):
		gripper_orientation = g_orient

    def setq(self,new_qi,new_q,new_qf):
	global qi, q, qf
	qi = new_qi
	q  = new_q
	qf = new_qf
		
    def setq_comp2(self,new_q_comp2):
	global q_comp2
	q_comp2 = new_q_comp2

    def setme(self, ui, rex):
	self.timerCallback = functools.partial(self.statemachine_check, ui = ui ,rex = rex)

    def getmestatus(self,combined=False):
	if(combined):
		return current_mode + ", " + current_action + ", " + current_movement + ", " + current_motorstate
	else:	
		return [current_mode, current_action, current_movement, current_motorstate]
   
    def setmystatus(self,new_mode="Don't set", new_action="Don't set", new_movement="Don't set", new_motorstate="Don't set"):
	global current_mode, current_action, current_motorstate, current_movement
	if(new_mode != "Don't set"):
		current_mode = new_mode
	if(new_action != "Don't set"):
		current_action = new_action
	if(new_movement != "Don't set"):
		current_movement = new_movement		
	if(new_motorstate != "Don't set"):
		current_motorstate = new_motorstate

    def motor_idle(self,rex):
	global current_motorstate	
	rex.speed_multiplier = [0.1]*rex.num_joints
	current_motorstate = "idle"
	rex.cmd_publish()
    
    def movement_idle(self):
	global current_movement	
	current_movement = "idle"

    def action_idle(self):
	global current_action	
	current_action = "idle"
		

    def mode_idle(self):
	global current_mode	
	current_mode = "idle"
	
    def estop(self, rex):
        #sets the torque of all motors to 0, in case of emergency
        rex.max_torque = [0.0]*rex.num_joints
        rex.cmd_publish()
        
    def checkmotors(self,rex):
	atol = 5.00 #do a formal way of selecting this tolerance
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
								#print "motor 5 fails" + str(abs(rex.joint_angles[5]*R2D-rex.joint_angles_fb[5]*R2D)) + "angles are: " + str(rex.joint_angles[5]*R2D) + str(rex.joint_angles_fb[5]*R2D)
								return False
						else:
							#print "motor 4 fails" + str(abs(rex.joint_angles[4]*R2D-rex.joint_angles_fb[4]*R2D)) + "angles are: " + str(rex.joint_angles[4]*R2D) + str(rex.joint_angles_fb[4]*R2D)
							return False
							
				else:
					#print "motor 3 fails" + str(abs(rex.joint_angles[3]*R2D-rex.joint_angles_fb[3]*R2D)) + "angles are: " + str(rex.joint_angles[3]*R2D) + str(rex.joint_angles_fb[3]*R2D)
					return False
			else:
				#print "motor 2 fails" + str(abs(rex.joint_angles[2]*R2D-rex.joint_angles_fb[2]*R2D)) + "angles are: " + str(rex.joint_angles[2]*R2D) + str(rex.joint_angles_fb[2]*R2D)
				return False
		else:
			#print "motor 1 fails" + str(abs(rex.joint_angles[1]*R2D-rex.joint_angles_fb[1]*R2D)) + "angles are: " + str(rex.joint_angles[1]*R2D) + str(rex.joint_angles_fb[1]*R2D)
			return False
	else:
		#print "motor 0 fails " + str(abs(rex.joint_angles[0]*R2D-rex.joint_angles_fb[0]*R2D)) + "angles are: " + str(rex.joint_angles[0]*R2D) + str(rex.joint_angles_fb[0]*R2D)
		return False 	


    
    def statemachine_check(self, ui, rex): #running at 100 ms
	global current_mode, current_action, current_movement, current_motorstate, past_states, comp1_status
	past_status = past_states[0]+", "+past_states[1]+", "+ past_states[2] + ", "+ past_states[3]
	if(current_motorstate == "motion"):
		if(self.checkmotors(rex)):
			self.motor_idle(rex)
	current_status = self.getmestatus(True)
	
	if(current_status != past_status and current_motorstate == "idle"): 
		print "current_status: " + current_status + "; past_status: " + past_status
		print "current_motorstate: " + current_motorstate		
		past_states = self.getmestatus(False)
	
		if(current_mode == "testing"):
			if(current_action == "picking"):			
				self.picking(ui,rex)		
			elif(current_action =="placing"):
				self.placing(ui,rex)
			elif(current_movement == "idle"):
				self.action_idle()		
				self.mode_idle()
		elif(current_mode == "Competition 1"):
			if(comp1_status == "idle"):
				comp1_status = 'blue'
						
			if(current_action=="idle"):							
				if (comp1_status=="blue"):
					self.setmystatus("Competition 1", "picking","picking")#mode="testing",action="picking")					
					comp1_status = "red"
					return 'red'
				elif(comp1_status=="red"):
					self.setmystatus("Competition 1", "picking","picking")#mode="testing",action="picking")	
					comp1_status = "red"					
					return 'green'
				elif(comp1_status == 'green'):
					comp1_status = "idle"
					self.mode_idle()
			else:
				self.picknplace(ui,rex)

		elif(current_mode == "Competition 2"):
			if(comp1_status == -1):
				comp1_status = 0
						
			if(current_action=="idle"):							
				if (comp1_status==0):
					self.setmystatus("Competition 2", "picking","picking")#mode="testing",action="picking")					
					comp1_status = 1
					return 'red'
				elif(comp1_status==1):
					self.setmystatus("Competition 2", "picking","picking")#mode="testing",action="picking")	
					comp1_status = 2					
					return 'green'
				elif(comp1_status == 2):
					comp1_status = -1
					self.mode_idle()
			else:
				self.picknplace(ui,rex)
		elif(current_mode == "Competition 3"):
			pass
		elif(current_mode == "Competition 4"):
			pass
		elif(current_mode == "Competition 5"):
			pass

	return "none"

    def getangles(self, color):
	blockx, blocky, blockz, angle = get_color_block_world_coord(color)
	endCoord = [(blockx)/10, (blocky)/10, (blockz+40)/10, gripper_orientation]	
	angles = inverseKinematics(endCoord[0],endCoord[1],endCoord[2],endCoord[3])
	angles[0] = round(self.trim_angle(angles[0]),2)
	angles[1] = round(self.trim_angle(angles[1]),2)	
	angles[2] = round(self.trim_angle(angles[2]),2)
	angles[3] = round(self.trim_angle(angles[3]),2)
	self.setq(angles,angles,angles)


    def picknplace(self,ui,rex):
	global current_movement, current_action

	if(current_action == "picking"):
		if(current_movement == "idle"):
			current_action = "placing"
			current_movement = "placing"
			self.set_placing_location()
			self.placing(ui,rex)
		else:
			self.picking(ui,rex)

	elif(current_action == "placing"):
		if(current_movement == "idle"):
			self.action_idle()
		else:
			self.placing(ui,rex)
	
    def set_placing_location(self):
	if(current_mode=="Competition 1"):
		new_q = [-q[0], q[1], q[2], q[3]]
	if(current_mode=="Competition 2"):
		new_q = [-q_comp2[0][comp2_status], q_comp2[1][comp2_status], q_comp2[2][comp2_status], q_comp2[3][comp2_status]]
	if(current_mode=="Competition 3"):
		new_q = [-q[0], q[1], q[2], q[3]]
	if(current_mode=="Competition 4"):
		new_q = [-q[0], q[1], q[2], q[3]]
	if(current_mode=="Competition 5"):
		new_q = [-q[0], q[1], q[2], q[3]]
	
	self.setq(new_q,new_q,new_q)		
	


    def picking(self,ui,rex):
	global current_movement
	if(current_movement == "picking"):
		#print "1"
		self.hold(ui,rex)
	elif(current_movement == "grabbing qi"):
		#print "2"		
		self.reach(ui,rex)
	elif(current_movement == "grabbing q"):
		#print "3"		
		self.close(rex)
	elif(current_movement == "shoulderlast grabbing q" or current_movement == "shoulderlast keeping q"):
		#print "4"		
		self.shoulderlast(rex)
	elif(current_movement == "keeping q"):
		#print "5"
		self.open(rex)
	elif(current_movement == "closing"):
		#print "6"		
		self.movement_idle()		
	else:
		pass#print "7:" + current_movement

    def placing(self,ui,rex):
	global current_movement
	if(current_movement == "placing"):
		self.hold(ui,rex)
	elif(current_movement == "keeping qi"):
		self.reach(ui,rex)
	elif(current_movement == "shoulderlast keeping q"):
		self.shoulderlast(rex)
	elif(current_movement == "keeping q"):
		self.open(rex)
	elif(current_movement == "opening"):
		current_movement = "shoulderfirst keeping qf"
		self.shoulderfirst(rex)
	elif(current_movement == "keeping qf"):
		self.hold(ui,rex) 	


	
    def hold(self,ui,rex):
	global current_motorstate, current_movement
	if(rex.num_joints==6):	
	
		if(current_action == "picking"):
			rex.joint_angles[5] = 90*D2R
			current_movement = "grabbing qi"
		else:
			rex.joint_angles[5] = -90*D2R
			if(current_movement == "keeping qf"):
				self.movement_idle()
			else:
				current_movement = "keeping qi"

		rex.joint_angles[4] = 0.0	
	rex.joint_angles[0] = 0.0
        rex.joint_angles[1] = 0.0
        rex.joint_angles[2] = 0.0
        rex.joint_angles[3] = 0.0
	current_motorstate = "motion" 
	rex.cmd_publish()

    def reach(self,ui,rex):
	global current_motorstate,current_movement
	rex.joint_angles[0] = q[0]*D2R #goal_angles[0]*D2R
	rex.joint_angles[2] = q[2]*D2R #goal_angles[2]*D2R
        rex.joint_angles[3] = q[3]*D2R #goal_angles[3]*D2R
	if(rex.num_joints==6):		
		rex.joint_angles[4] = wrist_orientation
	if(abs(q[0])>90):
		if(current_movement == "grabbing qi"):
			current_movement = "shoulderlast grabbing q"
		else:
			current_movement = "shoulderlast keeping q"	
	else:
	        rex.joint_angles[1] = q[1]*D2R  
		if(current_movement == "grabbing qi"):
			current_movement = "grabbing q"
		else:
			current_movement = "keeping q"
	current_motorstate = "motion"	
    	rex.cmd_publish()

    def basefirst(self,rex):
	global current_motorstate,current_movement
	rex.joint_angles[0] = q[0]*D2R
	current_motorstate = "motion"	
    	rex.cmd_publish()

    def shoulderlast(self,rex):
	global current_motorstate,current_movement
	rex.joint_angles[1] = q[1]*D2R
	if(current_movement=="shoulderlast grabbing q"):
		current_movement = "grabbing q"	
	elif(current_movement=="shoulderlast keeping q"):	
		current_movement = "keeping q"
	current_motorstate = "motion"		
	rex.cmd_publish()
	
    def shoulderfirst(self,rex):
	global current_motorstate,current_movement
	rex.joint_angles[1] = 0.0
	if(current_movement=="shoulderfirst grabbing qf"):
		current_movement = "grabbing qf"	
	elif(current_movement=="shoulderfirst keeping qf"):	
		current_movement = "keeping qf"
	current_motorstate = "motion"		
	rex.cmd_publish()


    def close(self,rex):
	global current_motorstate, current_movement
	#print "Hello from pick"
	if(rex.num_joints==6):	
		rex.joint_angles[5] = -90*D2R
	current_movement = "closing"   
	current_motorstate = "motion"	 		
	rex.cmd_publish()
    	
    def open(self,rex):
	global current_motorstate, current_movement
	if(rex.num_joints==6):	
		rex.joint_angles[5] = 90*D2R
	current_movement = "opening"
	current_motorstate = "motion"	
	rex.cmd_publish()
	
	
	    		
