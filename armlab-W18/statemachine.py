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
comp2_status = "idle" # idle, red, blue, green
comp3_status = "idle" # idle, red, blue, green
comp4_status = "idle" # idle, red, blue, green
comp5_status = "idle" # idle, red, blue, green
comp1 = -1 # 0,1,2,
comp2 = -1 # 0,1,2,
comp3 = -1 # 0,1,2,
comp4 = -1 # 0,1,2,
comp5 = -1 # 0,1,2,


past_status = ""
past_states = [""]*4
qi = [0.0]*6
qf = [0.0]*6
q  = [0.0]*6
#picking = "True"
#mode ="normal"
wrist_orientation = 0.0
gripper_orientation = 90.0
q_comp = np.zeros([4,3])
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
		
    def setq_comp(self,new_q_comp):
	global q_comp
	q_comp = new_q_comp

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
	global current_mode, current_action, current_movement, current_motorstate, past_states, comp1_status, comp2_status,comp3_status, comp4_status,comp5_status, comp1, comp2,comp3, comp4,comp5
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
					comp1_status = "green"					
					return 'green'
				elif(comp1_status == 'green'):
					comp1_status = "idle"
					self.mode_idle()
			else:
				self.picknplace(ui,rex)

		elif(current_mode == "Competition 2"):
			if(comp2 == -1):
				comp2 = 0
			print forwardKinematics(q[0],q[1],q[2],q[3])
			print comp2	
			if(current_action=="idle"):							
				if (comp2==0):
					self.setmystatus("Competition 2", "picking","picking")#mode="testing",action="picking")					
					comp2 = 1
					return 'red'
				elif(comp2 ==1):
					self.setmystatus("Competition 2", "picking","picking")#mode="testing",action="picking")	
					comp2 = 2					
					return 'green'
				elif(comp2 == 2):
					comp2 = -1
					self.mode_idle()
			else:
				self.picknplace(ui,rex)
		elif(current_mode == "Competition 3"):
			if(comp3_status == "idle"):
				comp3_status = 'blue'
				comp3 = 0		
			if(current_action=="idle"):							
				if (comp3_status=="blue"):
					self.setmystatus("Competition 3", "picking","picking")#mode="testing",action="picking")					
					comp3_status = "black"
					comp3 = 1
					return 'black'
				elif(comp3_status=="black"):
					self.setmystatus("Competition 3", "picking","picking")#mode="testing",action="picking")	
					comp3_status = "red"					
					comp3 = 2					
					return 'red'
				elif (comp3_status=="red"):
					self.setmystatus("Competition 3", "picking","picking")#mode="testing",action="picking")					
					comp3_status = "orange"
					comp3 = 3
					return 'orange'
				elif(comp3_status=="orange"):
					self.setmystatus("Competition 3", "picking","picking")#mode="testing",action="picking")	
					comp3_status = "yellow"					
					comp3 = 4					
					return 'yellow'
				elif(comp3_status == 'yellow'):
					self.setmystatus("Competition 3", "picking","picking")#mode="testing",action="picking")	
					comp3_status = "green"					
					comp3 = 5					
					return 'green'
				elif(comp3_status=="green"):
					self.setmystatus("Competition 3", "picking","picking")#mode="testing",action="picking")	
					comp3_status = "violet"					
					comp3 = 6					
					return 'violet'
				elif (comp3_status=="violet"):
					self.setmystatus("Competition 3", "picking","picking")#mode="testing",action="picking")					
					comp3_status = "pink"
					comp3 = 7
					return 'pink'
				elif(comp3_status == 'pink'):
					comp3_status = "idle"
					comp3 = -1	
					self.mode_idle()
			else:
				self.picknplace(ui,rex)
		elif(current_mode == "Competition 4"):
			if(comp4 == -1):
				comp4 = 0
				comp4_status = "blue"
			print forwardKinematics(q[0],q[1],q[2],q[3])
			print comp4	
			if(current_action=="idle"):							
				if (comp4_status=="blue"):
					self.setmystatus("Competition 4", "picking","picking")#mode="testing",action="picking")					
					comp4_status = "black"
					comp4 = 1
					return 'black'
				elif(comp4_status=="black"):
					self.setmystatus("Competition 4", "picking","picking")#mode="testing",action="picking")	
					comp4_status = "red"					
					comp4 = 2					
					return 'red'
				elif (comp4_status=="red"):
					self.setmystatus("Competition 4", "picking","picking")#mode="testing",action="picking")					
					comp4_status = "orange"
					comp4 = 3
					return 'orange'
				elif(comp4_status=="orange"):
					self.setmystatus("Competition 4", "picking","picking")#mode="testing",action="picking")	
					comp4_status = "yellow"					
					comp4 = 4					
					return 'yellow'
				elif(comp4_status == 'yellow'):
					self.setmystatus("Competition 4", "picking","picking")#mode="testing",action="picking")	
					comp4_status = "green"					
					comp4 = 5					
					return 'green'
				elif(comp4_status=="green"):
					self.setmystatus("Competition 4", "picking","picking")#mode="testing",action="picking")	
					comp4_status = "violet"					
					comp4 = 6					
					return 'violet'
				elif (comp4_status=="violet"):
					self.setmystatus("Competition 4", "picking","picking")#mode="testing",action="picking")					
					comp4_status = "pink"
					comp4 = 7
					return 'pink'
				elif(comp4_status == 'pink'):
					comp4_status = "idle"
					comp4 = -1	
					self.mode_idle()
			else:
				self.picknplace(ui,rex)
		elif(current_mode == "Competition 5"):
			pass

	return "none"


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
		new_q = [-q_comp[0][comp2], q_comp[1][comp2], q_comp[2][comp2], q_comp[3][comp2]]
	if(current_mode=="Competition 3"):
		new_q = [-q_comp[0][comp3], q_comp[1][comp3], q_comp[2][comp3], q_comp[3][comp3]]
	if(current_mode=="Competition 4"):
		new_q = [-q_comp[0][comp4], q_comp[1][comp4], q_comp[2][comp4], q_comp[3][comp4]]
	if(current_mode=="Competition 5"):
		new_q = [-q_comp[0][comp5], q_comp[1][comp5], q_comp[2][comp5], q_comp[3][comp5]]
	
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
	
	
	    		
