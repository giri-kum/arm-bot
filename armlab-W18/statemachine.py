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
current_wristpos = "closed"
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
close_angle = -60
open_angle = 90
past_status = ""
past_states = [""]*4
q  = [0.0]*6 #where last q[5] is gripper orientation should not be fed to last motor directly 
qh = [0.0]*6 #higher q
closing_threshold = abs(close_angle) - 18
#picking = "True"
#mode ="normal"
q_comp = np.zeros([3,6]) #3 blocks and 6 motors. Way to encode block position in terms of angles.
qh_comp = np.zeros([3,6]) #qh_comp- way to encode the intermediate location of the block before grabbing it.
debug = True
debug_motor= False
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

class Statemachine():
    timerCallback = functools.partial(gen_timestamp)
    
    def setq(self,new_q,new_qh = [0.0]*6):
	global q, qh
	q  = new_q
	qh = new_qh  
    """		
    def get_orientations(self,n=1):
	#print [gripper_orientation, wrist_orientation, gripping_height]
	if(n==1):
		return gripper_orientation
	elif(n==2):
		return wrist_orientation
	elif(n==3):
		return gripping_height

    def set_orientations(self,value,n=1):
	global gripper_orientation, wrist_orientation, gripper_height
	if(n==1):
		print "gripper_orientation changed from: " + str(gripper_orientation) + " to " + str(value)
		gripper_orientation = value
	elif(n==2):
		print "wrist_orientation changed from: " + str(wrist_orientation) + " to " + str(value)
		wrist_orientation = value
	elif(n==3):
		print "gripping_height changed from: " + str(gripping_height) + " to " + str(value)
		gripping_height = value
    """
    def setq_comp(self,new_q_comp,new_qh_comp):
	global q_comp,qh_comp
	q_comp = new_q_comp
	qh_comp = new_qh_comp

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
							if(isclose(rex.joint_angles[5]*R2D,rex.joint_angles_fb[5]*R2D,closing_threshold) and current_wristpos == "closed"):
								return True
							elif(isclose(rex.joint_angles[5]*R2D,rex.joint_angles_fb[5]*R2D,atol) and current_wristpos != "closed"):
								return True
							else:
								if(debug_motor):
									print "motor 5 fails" + str(abs(rex.joint_angles[5]*R2D-rex.joint_angles_fb[5]*R2D)) + "angles are: " + str(rex.joint_angles[5]*R2D) + str(rex.joint_angles_fb[5]*R2D)
								return False
						else:
							if(debug_motor):		
								print "motor 4 fails" + str(abs(rex.joint_angles[4]*R2D-rex.joint_angles_fb[4]*R2D)) + "angles are: " + str(rex.joint_angles[4]*R2D) + str(rex.joint_angles_fb[4]*R2D)
							return False
							
				else:
					if(debug_motor):
						print "motor 3 fails" + str(abs(rex.joint_angles[3]*R2D-rex.joint_angles_fb[3]*R2D)) + "angles are: " + str(rex.joint_angles[3]*R2D) + str(rex.joint_angles_fb[3]*R2D)
					return False
			else:
				if(debug_motor):
					print "motor 2 fails" + str(abs(rex.joint_angles[2]*R2D-rex.joint_angles_fb[2]*R2D)) + "angles are: " + str(rex.joint_angles[2]*R2D) + str(rex.joint_angles_fb[2]*R2D)
				return False
		else:
			if(debug_motor):
				print "motor 1 fails" + str(abs(rex.joint_angles[1]*R2D-rex.joint_angles_fb[1]*R2D)) + "angles are: " + str(rex.joint_angles[1]*R2D) + str(rex.joint_angles_fb[1]*R2D)
			return False
	else:
		if(debug_motor):
			print "motor 0 fails " + str(abs(rex.joint_angles[0]*R2D-rex.joint_angles_fb[0]*R2D)) + "angles are: " + str(rex.joint_angles[0]*R2D) + str(rex.joint_angles_fb[0]*R2D)
		return False 	


    
    def statemachine_check(self, ui, rex): #running at 100 ms
	global current_mode, current_action, current_movement, current_motorstate, past_states, comp1_status, comp2_status,comp3_status, comp4_status,comp5_status, comp1, comp2,comp3, comp4,comp5
	past_status = past_states[0]+", "+past_states[1]+", "+ past_states[2] + ", "+ past_states[3]
	if(current_motorstate == "motion"):
		if(self.checkmotors(rex)):
			self.motor_idle(rex)
	current_status = self.getmestatus(True)
	
	if(current_status != past_status and current_motorstate == "idle"): 
		if(debug):
			print "current_status: " + current_status + "; past_status: " + past_status
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
	#set_placing_location: for each competition, set_placing_location determines where to place the block.
	if(current_mode=="Competition 1"):
		new_q = [-q[0], q[1], q[2], q[3], q[4], q[5]]
		new_qh = [-qh[0], qh[1], qh[2], qh[3], qh[4], qh[5] ]		
	if(current_mode=="Competition 2"):
		#Competition 2: place blocks based on the mirrored position of the blue block.
		print q_comp
		new_q = [q_comp[comp2][0], q_comp[comp2][1], q_comp[comp2][2], q_comp[comp2][3],q_comp[comp2][4],q_comp[comp2][5]]
		new_qh = [qh_comp[comp2][0], qh_comp[comp2][1], qh_comp[comp2][2], qh_comp[comp2][3],qh_comp[comp2][4],q_comp[comp2][5]]
	if(current_mode=="Competition 3"):
		new_q = [-q_comp[0][comp3], q_comp[1][comp3], q_comp[2][comp3], q_comp[3][comp3]]
	if(current_mode=="Competition 4"):
		new_q = [q_comp[comp4][0], q_comp[comp4][1], q_comp[comp4][2], q_comp[comp4][3],q_comp[comp4][4],q_comp[comp4][5]]
		new_qh = [qh_comp[comp4][0], qh_comp[comp4][1], qh_comp[comp4][2], qh_comp[comp4][3],qh_comp[comp4][4],q_comp[comp4][5]]
	if(current_mode=="Competition 5"):
		new_q = [-q_comp[0][comp5], q_comp[1][comp5], q_comp[2][comp5], q_comp[3][comp5]]
	
	self.setq(new_q,new_qh)		

    def goingtomove(self,rex):
	global	current_movement, current_motorstate
	rex.joint_angles[0] = qh[0]*D2R
	rex.joint_angles[1] = qh[1]*D2R
	rex.joint_angles[2] = qh[2]*D2R
	rex.joint_angles[3] = qh[3]*D2R
	if(current_action == "picking"):
		current_movement = "grabbing q"
	elif(current_action == "placing"):
		current_movement = "shoulderfirst keeping qf"
	current_motorstate = "motion"		
	rex.cmd_publish()


    def picking(self,ui,rex):
	global current_movement
	
	if(current_movement == "picking"):
		#print "1"
		if(abs(q[5]-180) < 0.001): # interchange q and qh
			self.setq(qh,q)
		self.hold(ui,rex)
	elif(current_movement == "grabbing qi"):
		#print "2"		
		self.reach(ui,rex)
	elif(current_movement == "shoulderlast grabbing q"):
		#print "3"		
		self.shoulderlast(rex)
	elif(current_movement == "going to grab"):
		#print "4"		
		self.goingtomove(rex)
	elif(current_movement == "grabbing q"):
		#print "5"		
		self.close(rex)
	elif(current_movement == "closing"):
		#print "6"
		if(abs(q[5]-180) < 0.001): # interchange q and qh back
			self.setq(qh,q)			
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
		if(abs(q[5]-180) < 0.001):
			current_movement = "going to leave"
			self.goingtomove(rex)
		else:
			current_movement = "shoulderfirst keeping qf"
			self.shoulderfirst(rex)
	elif(current_movement == "shoulderfirst keeping qf"):
		self.shoulderfirst(rex)
	elif(current_movement == "keeping qf"):
		self.hold(ui,rex) 	


	
    def hold(self,ui,rex):
	global current_motorstate, current_movement
	if(rex.num_joints==6):	
		if(current_action == "picking"):
			rex.joint_angles[5] = open_angle*D2R
			current_movement = "grabbing qi"
		else:
			rex.joint_angles[5] = close_angle*D2R
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
		rex.joint_angles[4] = q[4]*D2R
	#if(abs(q[0])>90):
	if(current_movement == "grabbing qi"):
		current_movement = "shoulderlast grabbing q"
	else:
		current_movement = "shoulderlast keeping q"	
	"""
	else:
	        rex.joint_angles[1] = q[1]*D2R  
		if(current_movement == "grabbing qi"):
			current_movement = "grabbing q"
		else:
			current_movement = "keeping q"
	"""	
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
		if(abs(q[5]-180) < 0.001):
			current_movement = "going to grab"
		else:
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
	global current_motorstate, current_movement, current_wristpos
	#print "Hello from pick"
	if(rex.num_joints==6):	
		rex.joint_angles[5] = close_angle*D2R
	current_movement = "closing"
	current_wristpos = "closed"   
	current_motorstate = "motion"	 		
	rex.cmd_publish()
    	
    def open(self,rex):
	global current_motorstate, current_movement, current_wristpos
	if(rex.num_joints==6):	
		rex.joint_angles[5] = open_angle*D2R
	current_movement = "opening"
	current_wristpos = "opened"
	current_motorstate = "motion"	
	rex.cmd_publish()
	
	
	    		
