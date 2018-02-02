import datetime #Giri
import time
import numpy as np
import functools
import decimal
import pylab

waypoints = [0.0,0.0,0.0,0.0]*2
viapoints = [0.0,0.0,0.0,0.0]
viaspeed =  [0.0,0.0,0.0,0.0]
datasheet_speed = [55.0,55.0,55.0,59.0,114.0,114.0] #in rpm [0.33,0.33,0.33,0.531,0.684,0.684] degrees per millisecond. #[0.263s] for 180 
max_speed = [datasheet_speed[i]/2 for i in range(0,6)]
""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592
ToRPM = 60.0/360 #Degree/s to rpm
max_i = 0
folder = "lessons/"	
filename = "Now"
current_mode = "idle" # teach, repeat, spline repeat
current_action = "idle" # learning, repeating, computing, spline repeating
current_motorstate = "idle" #motion
current_i = 0
past_status = ""
past_states = [""]*4
global_tf = 10 #in seconds
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

    def checkmotors(self,rex,setangles=["hi",0,0,0],atol = [5.000,5.000,5.000,5.000]):
	if(setangles[0] == "hi"):
		setangles = [rex.joint_angles[0]*R2D,rex.joint_angles[1]*R2D,rex.joint_angles[2]*R2D,rex.joint_angles[3]*R2D]
	
	 #do a formal way of selecting this tolerance
	if (isclose(setangles[0],rex.joint_angles_fb[0]*R2D,atol[0])):
		if(isclose(setangles[1],rex.joint_angles_fb[1]*R2D,atol[1])):
			if(isclose(setangles[2],rex.joint_angles_fb[2]*R2D,atol[2])):
				if(isclose(setangles[3],rex.joint_angles_fb[3]*R2D,atol[3])):
					return True
				else:
					print "motor 3 fails" + str(abs(setangles[3]-rex.joint_angles_fb[3]*R2D))
					return False
			else:
				print "motor 2 fails" + str(abs(setangles[2]-rex.joint_angles_fb[2]*R2D))
				return False
		else:
			print "motor 1 fails" + str(abs(setangles[1]-rex.joint_angles_fb[1]*R2D))
			return False
	else:
		print "motor 0 fails " + str(abs(setangles[0]-rex.joint_angles_fb[0]*R2D))
		return False 	

    
    def statemachine_check(self, ui, rex): #running at 100 ms
	global current_mode, current_action, current_i, current_motorstate, past_states, max_i
	past_status = past_states[0]+", "+past_states[1]+", "+ str(past_states[2])+", "+ past_states[3]

	if(current_motorstate == "motion"):
		if(current_action == "going to wp2"):
			vp = [viapoints[0,current_i+1],viapoints[1,current_i+1],viapoints[2,current_i+1],viapoints[3,current_i+1]]
			tol = [20, 20, 20, 20]
			if(self.checkmotors(rex,vp,tol)):
				self.motor_idle(rex)
		else:
			if(self.checkmotors(rex)):
				self.motor_idle(rex)
	current_status = self.getmestatus(True)
	ui.rdoutStatus.setText(current_status)		

	if(current_mode == "repeat" and current_action == "repeating" and current_motorstate == "idle"):
		current_i = current_i+1
		if(current_i >= max_i):
			self.mode_idle()
			self.action_idle()
			ui.btnUser5.setText("Repeat")
			ui.btnUser5.setEnabled(True)
			ui.btnUser8.setEnabled(True)
		else:		
			self.set_into_motion(ui,rex)
	if(current_status != past_status and current_motorstate == "idle"): 
		print "current_status: " + current_status + "; past_status: " + past_status
		print "current_motorstate: " + current_motorstate		
		past_states = self.getmestatus(False)
		
		if(current_mode == "spline repeat"):
			if(current_action == "compute"):
				self.srepeat(ui,rex)
			elif(current_action == "go to wp1"):
				self.sreset(ui,rex)
			elif(current_action == "ready"):
				self.launch(ui,rex)
			elif(current_action == "going to wp2"):
				current_i = current_i+1
				print "max_i=" + str(max_i) 
				if(current_i >=max_i-1):
					self.mode_idle()
					self.action_idle()
					ui.btnUser8.setText("Smooth Repeat")
					ui.btnUser5.setEnabled(True)
					ui.btnUser8.setEnabled(True)
				else:		
					self.launch(ui,rex)
			
		else:
			pass #print current_state

    def launch(self,ui,rex):
	global current_motorstate, current_action 	
	rex.torque_multiplier = ui.sldrMaxTorque.value()/100.0
	current_action = "going to wp2"
	rex.joint_angles[0]= viapoints[0,max_i-1]*D2R
	rex.joint_angles[1]= viapoints[1,max_i-1]*D2R
	rex.joint_angles[2]= viapoints[2,max_i-1]*D2R
	rex.joint_angles[3]= viapoints[3,max_i-1]*D2R        	
	rex.speed_multiplier[0] = viaspeed[0,current_i+1]
        rex.speed_multiplier[1] = viaspeed[1,current_i+1]
        rex.speed_multiplier[2] = viaspeed[2,current_i+1]
        rex.speed_multiplier[3] = viaspeed[3,current_i+1]
	rex.cmd_publish()
	print "point: " + str(i)
	print rex.speed_multiplier
	current_motorstate = "motion"
	
    def sreset(self,ui,rex):
	global current_motorstate, current_action	
	rex.torque_multiplier = ui.sldrMaxTorque.value()/100.0
	rex.joint_angles[0]= float(viapoints[0,0])*D2R
	rex.joint_angles[1]= float(viapoints[1,0])*D2R
	rex.joint_angles[2]= float(viapoints[2,0])*D2R
	rex.joint_angles[3]= float(viapoints[3,0])*D2R        	
	rex.speed_multiplier[0] = ui.sldrSpeed.value()/100.0
        rex.speed_multiplier[1] = ui.sldrSpeed.value()/100.0
        rex.speed_multiplier[2] = ui.sldrSpeed.value()/100.0
        rex.speed_multiplier[3] = ui.sldrSpeed.value()/100.0        		
	rex.cmd_publish()
	current_motorstate = "motion"
	current_action = "ready"


    def srepeat(self, ui, rex):
	global current_i, max_i, wp_lines, current_action, current_mode, waypoints, viapoints, viaspeed 
	current_mode = "spline repeat"	
	current_action = "computing"	
	ui.btnUser5.setEnabled(False) 				
	ui.btnUser8.setEnabled(False) 						
	ui.btnUser8.setText("Repeating Spline")
	f = open(folder+"wp_"+filename+".csv", "r")
	wp_lines = f.readlines()
	f.close()		
	max_i = len(wp_lines)		
	tf = int(global_tf*1000)
	
	if(max_i!=2):
		print "Enter only two way points"
	else:
		q = np.zeros([4,max_i])
		wp = wp_lines[0].split(',')
		q[0,0]= float(wp[0])
		q[1,0]= float(wp[1])
		q[2,0]= float(wp[2])
		q[3,0]= float(wp[3])        					
		wp = wp_lines[1].split(',')
		q[0,1]= float(wp[0])
		q[1,1]= float(wp[1])
		q[2,1]= float(wp[2])
		q[3,1]= float(wp[3])
		t,sq0,sv0,aq0 = self.computespline(q[0,1],q[0,0],0,tf)
		t,sq1,sv1,aq1 = self.computespline(q[1,1],q[1,0],1,tf)
		t,sq2,sv2,aq2 = self.computespline(q[2,1],q[2,0],2,tf)
		t,sq3,sv3,aq3 = self.computespline(q[3,1],q[3,0],3,tf)
		sq = np.concatenate(([sq0],[sq1],[sq2],[sq3]))	
		sv = np.concatenate(([sv0],[sv1],[sv2],[sv3]))	
		aq = np.concatenate(([aq0],[aq1],[aq2],[aq3]))	
		self.savedata(t,sq,sv)
		self.trajectory()
		waypoints = q
		viapoints = sq
		viaspeed = sv
		current_i = 0
		max_i = len(sq0)
		current_action = "go to wp1"

    def setme(self, ui, rex):
	self.timerCallback = functools.partial(self.statemachine_check, ui = ui ,rex = rex)
   
    def motor_idle(self,rex):
	global current_motorstate	
	rex.speed_multiplier = [0.1]*rex.num_joints
	current_motorstate = "idle"
	rex.cmd_publish()

    def action_idle(self):
	global current_action	
	current_action = "idle"
		
    def mode_idle(self):
	global current_mode	
	current_mode = "idle"

    def i_idle(self):
	global current_i	
	current_i = 0

    def estop(self, rex):
	global current_motorstate	
	current_motorstate = "estop"
        #sets the torque of all motors to 0, in case of emergency
        rex.max_torque = [0.0, 0.0, 0.0, 0.0]
        rex.cmd_publish()

    def getmestatus(self,combined=False):
	if(combined):
		return current_mode + ", " + current_action + ", " + str(current_i) + ", " + current_motorstate
	else:	
		return [current_mode, current_action, current_i, current_motorstate]
   
    def setmystatus(self,new_mode="Don't set", new_action="Don't set", new_motorstate="Don't set", new_i = "Don't set"):
	global current_mode, current_action, current_motorstate, current_i
	if(new_mode != "Don't set"):
		current_mode = new_mode
	if(new_action != "Don't set"):
		current_action = new_action
	if(new_motorstate != "Don't set"):
		current_motorstate = new_motorstate
	if(new_i != "Don't set"):
		current_i = new_motorstate

    def teach(self,ui,rex):
	global filename, current_mode, current_action
	if(ui.btnUser4.text() == "Teach"):	
		current_mode = "teach"
		current_action = "learning"
		ui.btnUser5.setEnabled(True)
		ui.btnUser8.setEnabled(False) 						
		ui.btnUser4.setText("Stop Learning")
		ui.btnUser5.setText("Learn This")
		filename = gen_timestamp()
		rex.torque_multiplier = 0;
		rex.cmd_publish()

	elif(ui.btnUser4.text() == "Stop Learning"):
		self.mode_idle()
		self.action_idle()
		ui.btnUser4.setText("Teach")
		ui.btnUser5.setText("Repeat")
		ui.btnUser8.setEnabled(True)
	
    def repeat(self,ui,rex): #Giri
	global current_i, max_i, wp_lines, current_action, current_mode
	current_i = 0
	
	if(ui.btnUser4.text() == "Learn"):	
		ui.btnUser4.setText("Teach")
	if(ui.btnUser5.text() == "Learn This"):	
		f = open(folder+"wp_"+filename+".csv", "a")
		waypoints[0] = rex.joint_angles_fb[0]*R2D
		waypoints[1] = rex.joint_angles_fb[1]*R2D
		waypoints[2] = rex.joint_angles_fb[2]*R2D
		waypoints[3] = rex.joint_angles_fb[3]*R2D
		f.writelines(str(waypoints[0])+ ',' + str(waypoints[1])+ ',' + str(waypoints[2])+ ',' + str(waypoints[3]) + '\n')
		f.close()
	
	elif(ui.btnUser5.text() == "Repeat"):
		current_mode = "repeat"	
		current_action = "computing"	
		ui.btnUser5.setEnabled(False) 				
		ui.btnUser8.setEnabled(False) 						
		f = open(folder+"wp_"+filename+".csv", "r")
		wp_lines = f.readlines()
		f.close()		
		max_i = len(wp_lines)
		print "max_i:" + str(max_i)
		ui.btnUser5.setText("Repeating")
		current_action = "repeating"
		self.set_into_motion(ui,rex)			


    def savedata(self,t,sq,sv):
	f = open(folder+"wp_"+filename+"_c.csv", "a")
	g = open(folder+"sp_"+filename+"_c.csv", "a")
	for i in range(0,len(t)):
		f.writelines(str(sq[0,i])+ ',' + str(sq[1,i])+ ',' + str(sq[2,i])+ ',' + str(sq[3,i]) + ',' + str(t[i]) + '\n')
		g.writelines(str(sv[0,i])+ ',' + str(sv[1,i])+ ',' + str(sv[2,i])+ ',' + str(sv[3,i]) + ',' + str(t[i]) + '\n')		
	g.close()		
	f.close()
	
    def getdata(self):
	f = open(folder+"wp_"+filename+"_c.csv", "r")
	g = open(folder+"sp_"+filename+"_c.csv", "r")
	wp_lines = f.readlines()
	sp_lines = g.readlines()			
	g.close()
	f.close()
	max_i = len(wp_lines)
	q = np.zeros([4,max_i])
	v = np.zeros([4,max_i])
	t = np.zeros([max_i])
	for i in range(0,max_i):
		waypoints = wp_lines[i].split(',')
		speedpoints = sp_lines[i].split(',')
		q[0,i]= float(waypoints[0])
		q[1,i]= float(waypoints[1])
		q[2,i]= float(waypoints[2])
		q[3,i]= float(waypoints[3])        					
		v[0,i]= float(speedpoints[0])
		v[1,i]= float(speedpoints[1])
		v[2,i]= float(speedpoints[2])
		v[3,i]= float(speedpoints[3])        					
		t[i]  = float(speedpoints[4])
	return [t,q,v]

				
    def set_into_motion(self,ui,rex):
	global current_motorstate	
	print "Entered: " + str(current_i)
	wp_points=wp_lines[current_i].split(',')
	rex.torque_multiplier = ui.sldrMaxTorque.value()/100.0
	rex.joint_angles[0]= float(wp_points[0])*D2R
	rex.joint_angles[1]= float(wp_points[1])*D2R
	rex.joint_angles[2]= float(wp_points[2])*D2R
	rex.joint_angles[3]= float(wp_points[3])*D2R        	
	rex.speed_multiplier[0] = ui.sldrSpeed.value()/100.0
        rex.speed_multiplier[1] = ui.sldrSpeed.value()/100.0
        rex.speed_multiplier[2] = ui.sldrSpeed.value()/100.0
        rex.speed_multiplier[3] = ui.sldrSpeed.value()/100.0        		
	current_motorstate = "motion"
        rex.cmd_publish()


    
    def compute_via_points(self,ui,rex):
	global current_action	
	current_action = "computing"

	f = open(folder+"wp_"+filename+".csv", "r")
	wp_lines = f.readlines()
	max_i = len(wp_lines)	
	f.close()
	wp = np.zeros([4,max_i])
	sp = np.zeros([4,max_i])
	
	for i in range(0,max_i):
		waypoints = wp_lines[i].split(',')
		speedpoints = sp_lines[i].split(',')
		wp[0,i]= float(waypoints[0]) #Note that you are storing here ulta. Not the normal convention
		wp[1,i]= float(waypoints[1])
		wp[2,i]= float(waypoints[2])
		wp[3,i]= float(waypoints[3])        					
		sp[0,i]= float(speedpoints[0])
		sp[1,i]= float(speedpoints[1])
		sp[2,i]= float(speedpoints[2])
		sp[3,i]= float(speedpoints[3])        					
		
	f = open(folder+"wp_"+filename+"_c.csv", "a")
	g = open(folder+"sp_"+filename+"_c.csv", "a")	
	for i in range(0,max_i):
		waypoints[0] = wp[0,i]; #local variable waypoints
		waypoints[1] = wp[1,i];
		waypoints[2] = wp[2,i];
		waypoints[3] = wp[3,i];
		speedpoints[0] = sp[0,i];
		speedpoints[1] = sp[1,i];
		speedpoints[2] = sp[2,i];
		speedpoints[3] = sp[3,i];
		f.writelines(str(waypoints[0])+ ',' + str(waypoints[1])+ ',' + str(waypoints[2])+ ',' + str(waypoints[3]) + '\n')
		g.writelines(str(speedpoints[0])+ ',' + str(speedpoints[1])+ ',' + str(speedpoints[2])+ ',' + str(speedpoints[3]) + '\n')
	g.close()
	f.close()
	
    
    def computespline(self,qf,q0,motor,tf=1000,tstep=100,t0=0,vf=0,v0=0): #time is in milliseconds, tstep 200 mseconds communication is 15 Hz so need min. 66 ms.
	t = [x/1000.0 for x in range(t0,tf,tstep)] 	
	N = len(t)	
	conversion_factor = ToRPM/max_speed[motor] #to percentage of the max allowable speed of the motor
	new_q = np.zeros(N)	
	v = np.zeros(N)
	b = np.array([q0, v0, qf, vf]).transpose()
	M = np.array([[1, t[0], t[0]*t[0], t[0]*t[0]*t[0]], [0, 1, 2*t[0], 3*t[0]*t[0]], [1, t[-1], t[-1]*t[-1], t[-1]*t[-1]*t[-1]], [0, 1, 2*t[-1], 3*t[-1]*t[-1]]])
	a = np.matmul(np.linalg.inv(M),b)
	for i in range(0,N):
		new_q[i] = a[0] + a[1]*t[i] + a[2]*t[i]*t[i] + a[3]*t[i]*t[i]*t[i]
		v[i] = abs(a[1] + 2*a[2]*t[i] + 3*a[3]*t[i]*t[i])*conversion_factor 
		
	return [t,new_q,v,a]
	
    def trajectory(self):
	f = open(folder+"wp_"+filename+"_c.csv", "r")
	g = open(folder+"sp_"+filename+"_c.csv", "r")
	sq_lines = f.readlines()		
	sv_lines = g.readlines()				
	g.close()		
	f.close()

	max_i = len(sq_lines)
	sq = np.zeros([4,max_i])
	sv = np.zeros([4,max_i])
	stq = np.zeros(max_i)
	stv = np.zeros(max_i)

	for i in range(0,max_i):	
		sqpoints = sq_lines[i].split(',')
		sq[0,i]= float(sqpoints[0])
		sq[1,i]= float(sqpoints[1])
		sq[2,i]= float(sqpoints[2])
		sq[3,i]= float(sqpoints[3])
		stq[i]  = float(sqpoints[4])

	for i in range(1,max_i):
		svpoints = sv_lines[i].split(',')
		sv[0,i]= float(svpoints[0])
		sv[1,i]= float(svpoints[1])
		sv[2,i]= float(svpoints[2])
		sv[3,i]= float(svpoints[3])
		stv[i] = float(svpoints[4])

	f, (ax1,ax2) = pylab.subplots(2, sharex=True)    #Uncomment for a new figure plot
	
	ax1.plot(stq,sq[0],'bo--',label='set base angle')	
	ax1.plot(stq,sq[1],'gx--',label='set shoulder angle')	
	ax1.plot(stq,sq[2],'r+--',label='set elbow angle')	
	ax1.plot(stq,sq[3],'mv--',label='set wrist angle')	

	box = ax1.get_position()
	ax1.set_position([box.x0, box.y0, box.width * 0.8, box.height])
	ax1.legend(loc='center left', bbox_to_anchor=(1, 0.5))	
	ax1.set_title('Motor Feedback and Set Waypoints (in degrees)')

	ax2.plot(stv,sv[0],'bo--',label='set base speed')	
	ax2.plot(stv,sv[1],'gx--',label='set shoulder speed')	
	ax2.plot(stv,sv[2],'r+--',label='set elbow speed')	
	ax2.plot(stv,sv[3],'mv--',label='set wrist speed')

	box = ax2.get_position()
	ax2.set_position([box.x0, box.y0, box.width * 0.8, box.height])
	ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5))	
	ax2.set_title('Estimated and Set Speed (in percentage)')

	pylab.show()


