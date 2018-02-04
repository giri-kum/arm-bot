#!/usr/bin/env python  --- COPY ----
import sys
import cv2
import numpy as np
from kinematics import *
from PyQt4 import QtGui, QtCore, Qt
from ui import Ui_MainWindow
from rexarm import Rexarm
from video import Video
from statemachine import Statemachine

debug = True

""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592

""" Pyxel Positions of image in GUI """
MIN_X = 310
MAX_X = 950

MIN_Y = 30
MAX_Y = 510

putx = 0.0
puty = 0.0
putz = 0.0
putangle = 0.0

gripper_orientation = 180.0

class Gui(QtGui.QMainWindow):
    """ 
    Main GUI Class
    contains the main function and interfaces between 
    the GUI and functions
    """
    def __init__(self,parent=None):
	
        QtGui.QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        """ Main Variables Using Other Classes"""
        self.rex = Rexarm()
        self.video = Video()
        self.statemachine = Statemachine()
	
	#self.video.extrinsic = np.array(self.video.getcaldata())
	#self.video.inv_extrinsic = np.array(np.linalg.inv(self.video.extrinsic))
        """ Other Variables """
        self.last_click = np.float32([0,0])

        """ Set GUI to track mouse """
        QtGui.QWidget.setMouseTracking(self,True)
	self.statemachine.setme(self.ui,self.rex)
        
        """ 
        GUI timer 
        Creates a timer and calls update_gui() function 
        according to the given time delay (27ms) 
        """
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.update_gui)
        self._timer.start(27)
       
        """ 
        LCM Arm Feedback timer
        Creates a timer to call LCM handler for Rexarm feedback
        """  
        self._timer2 = QtCore.QTimer(self)
        self._timer2.timeout.connect(self.rex.get_feedback)
        self._timer2.start()

	"""
	Giri - Statemachine timer
	"""
	#self._timer3 = QtCore.QTimer(self)
        #self._timer3.timeout.connect(self.statemachine.timerCallback)
        #self._timer3.start(100)  # frequency of the timer is set by this. it is 100 ms now. However, the time.now can give microseconds precision not just millisecond
	
	
        """ 
        Connect Sliders to Function
        TODO: CONNECT THE OTHER 5 SLIDERS IMPLEMENTED IN THE GUI 
        """ 
        self.ui.sldrBase.valueChanged.connect(self.sliderBaseChange)
        self.ui.sldrShoulder.valueChanged.connect(self.sliderShoulderChange)     #Giri
        self.ui.sldrElbow.valueChanged.connect(self.sliderElbowChange) 	 #Giri
        self.ui.sldrWrist.valueChanged.connect(self.sliderWristChange)        #Giri
	self.ui.sldrGrip1.valueChanged.connect(self.sliderGrip1Change)        #Giri
        self.ui.sldrGrip2.valueChanged.connect(self.sliderGrip2Change)        #Giri        
	self.ui.sldrMaxTorque.valueChanged.connect(self.sliderOtherChange)        #Giri
        self.ui.sldrSpeed.valueChanged.connect(self.sliderOtherChange)        #Giri        
	if(self.rex.num_joints==6):		
		self.ui.sldrGrip1.setEnabled(True) #Giri
		self.ui.sldrGrip2.setEnabled(True) #Giri
	else:
		self.ui.sldrGrip1.setEnabled(False) #Giri
		self.ui.sldrGrip2.setEnabled(False) #Giri
		
        """ Initial command of the arm to home position"""
        self.sliderChange() 
        self.reset()
        """ Connect Buttons to Functions 
        TODO: NAME AND CONNECT BUTTONS AS NEEDED
        """
        self.ui.btnUser1.setText("Configure Servos")
        self.ui.btnUser1.clicked.connect(self.btn1) #Giri

	self.ui.btnUser2.setText("Depth and RGB Calibration")
	self.ui.btnUser2.clicked.connect(self.btn2) #Giri
	self.ui.btnUser2.setEnabled(False) 	

        self.ui.btnUser3.setText("Extrinsic Calibration") #Giri
	self.ui.btnUser3.clicked.connect(self.btn3)

	self.ui.btnUser4.setText("Block Detector") #Giri
	self.ui.btnUser4.clicked.connect(self.btn4)

	self.ui.btnUser5.setText("Repeat")
	self.ui.btnUser5.clicked.connect(self.btn5) #Giri
	
	self.ui.btnUser6.setText("Teach")
	self.ui.btnUser6.clicked.connect(self.teach) #Giri

        self.ui.btnUser7.setText("Smooth Repeat")
	self.ui.btnUser7.clicked.connect(self.srepeat) #Giri        
	self.ui.btnUser7.setEnabled(False) #Giri			

	self.ui.btnUser8.setText("Pick Block")
	self.ui.btnUser8.clicked.connect(self.pick)

	self.ui.btnUser9.setText("Place Block")
	self.ui.btnUser9.clicked.connect(self.place)
	
	self.ui.btnUser10.setText("Reset")
	self.ui.btnUser10.clicked.connect(self.reset) #Giri

	self.ui.btnUser11.setText("Enter Competition Mode")
	self.ui.btnUser11.clicked.connect(self.competition) #Giri	

	self.ui.btnUser12.setText("Emergency Stop!")
        self.ui.btnUser12.clicked.connect(self.estop)



    def estop(self):
        self.statemachine.estop(self.rex)

    def trim_angle(self,q,motorno = 1): #for 1,2,3
	if(motorno > 2):
		max_angle = 300
	else:
		max_angle = 360

	if(q>max_angle or q<-1*max_angle):
		new_q = 0
		print "trimmed: " + str(q) + " motor no: " + str(motorno)
	elif(q>180): #181 --> -179
		new_q = q-360
		print "trimmed: " + str(q) + " motor no: " + str(motorno)
	elif(q<-180):
		new_q = 360-q
		print "trimmed: " + str(q) + " motor no: " + str(motorno)
	else :
		new_q = q
	return new_q

    def shortorientation(self,w_angle):
	if(w_angle>90):
		sw_angle = w_angle - 90
	elif(w_angle<-90):
		sw_angle = w_angle + 90
	else:
		sw_angle = w_angle	
	return sw_angle



    def update_gui(self):
        """ 
        update_gui function
        Continuously called by timer1 
        """
	
        """ Renders the video frames
            Displays a dummy image if no video available
            HINT: you can use the radio buttons to select different
            video sources like is done below
        """
        color = self.statemachine.timerCallback()
	if(color != "none"):
	    self.getangles(color)	
        if(self.video.kinectConnected == 1):
            try:
                self.video.captureVideoFrame()
                self.video.captureDepthFrame()
            except TypeError:
                self.video.kinectConnected = 0
                self.video.loadVideoFrame()
                self.video.loadDepthFrame()

        if(self.ui.radioVideo.isChecked()):
            self.ui.videoFrame.setPixmap(self.video.convertFrame())

        if(self.ui.radioDepth.isChecked()):
            self.ui.videoFrame.setPixmap(self.video.convertDepthFrame())

        
        """ 
        Update GUI Joint Coordinates Labels
        """
        self.ui.rdoutBaseJC.setText(str("%.2f" % (self.rex.joint_angles_fb[0]*R2D)))
        self.ui.rdoutShoulderJC.setText(str("%.2f" % (self.rex.joint_angles_fb[1]*R2D)))
        self.ui.rdoutElbowJC.setText(str("%.2f" % (self.rex.joint_angles_fb[2]*R2D)))
        self.ui.rdoutWristJC.setText(str("%.2f" % (self.rex.joint_angles_fb[3]*R2D)))
	self.ui.rdoutGrip1.setText(str(self.ui.sldrGrip1.value() )) #Giri
        self.ui.rdoutGrip2.setText(str(self.ui.sldrGrip2.value() )) #Giri
        

	"""
        Update End Effector Coordinates Kevin
	"""        
	endCoord=forwardKinematics(self.rex.joint_angles_fb[0]*R2D,self.rex.joint_angles_fb[1]*R2D,self.rex.joint_angles_fb[2]*R2D,self.rex.joint_angles_fb[3]*R2D)
        self.ui.rdoutX.setText(str("%2f" %(round(endCoord[0],2)) ) )
        self.ui.rdoutY.setText(str("%2f" %(round(endCoord[1],2)) ) )
        self.ui.rdoutZ.setText(str("%2f" %(round(endCoord[2],2)) ) )
        self.ui.rdoutT.setText(str("%2f" %(round(endCoord[3],2)) ) )
	
        """ 
        Mouse position presentation in GUI
        TODO: after implementing workspace calibration 
        display the world coordinates the mouse points to 
        in the RGB video image.
        """    
        x = QtGui.QWidget.mapFromGlobal(self,QtGui.QCursor.pos()).x()
        y = QtGui.QWidget.mapFromGlobal(self,QtGui.QCursor.pos()).y()
        if ((x < MIN_X) or (x > MAX_X) or (y < MIN_Y) or (y > MAX_Y)):
            self.ui.rdoutMousePixels.setText("(-,-,-)")
            self.ui.rdoutMouseWorld.setText("(-,-,-)")
        else:
            x = x - MIN_X
            y = y - MIN_Y
	    z = self.video.currentDepthFrame[y][x]
            self.ui.rdoutMousePixels.setText("(%.0f,%.0f,%.0f)" % (x,y,z))
            if (self.video.cal_flag == 2):
                M = self.video.aff_matrix
                v = np.float32([x,y,1])
                rw = np.dot(M,v)
                self.ui.rdoutMouseWorld.setText("(%.0f,%.0f,0)" % (rw[0],rw[1]))
	    elif (self.video.extrinsic_cal_flag == 2): #Josh
	        xy_in_rgb = np.float32([[[x,y]]]) #Josh
		xy_in_rgb_homogenous = np.array([x,y,1.0])
	        self.video.rgb2dep_aff_matrix = np.array([[1.09403672e0, 3.44369111e-3, -1.62867595e0],[-2.41966785e-3, 1.07534829e0,-2.69041712e1],[0., 0., 1.]]) #Josh
	        xy_in_dep = np.dot(self.video.rgb2dep_aff_matrix, xy_in_rgb_homogenous) #Josh
	        x_dep = int(xy_in_dep[0]) #Josh
	        y_dep = int(xy_in_dep[1]) #Josh
                d = self.video.currentDepthFrame[y_dep][x_dep]
		Z = -1
		#Z = 0.1236*np.tan(d/2842.5 + 1.1863)*1000
		for idx in range(len(self.video.levels_mm)):
		    if d <=self.video.levels_upper_d[idx] and d>=self.video.levels_lower_d[idx]:
			Z = 941 - self.video.levels_mm[idx]
			break
		camera_coord = Z*cv2.undistortPoints(xy_in_rgb,self.video.intrinsic,self.video.distortion_array)
		camera_coord_homogenous = np.transpose(np.append(camera_coord,[Z,1.0]))
		world_coord_homogenous = np.dot(self.video.extrinsic, camera_coord_homogenous)
		
		#camera_coord = Z*np.dot(self.video.inv_intrinsic,xy_in_rgb)
		#camera_coord_homo = np.concatenate((camera_coord,[1.]),axis = 0)
		#world_coord_homo = np.dot(self.video.extrinsic,camera_coord_homo)
		self.ui.rdoutMouseWorld.setText("(%.0f,%.0f,%.0f)" % (world_coord_homogenous[0],world_coord_homogenous[1],world_coord_homogenous[2]))
		#self.ui.rdoutMouseWorld.setText("(%.0f,%.0f,%.0f)" % (x_dep,y_dep,d))
            else:
                self.ui.rdoutMouseWorld.setText("(-,-,-)")

        """ 
        Updates status label when rexarm playback is been executed.
        This can be extended to include other appropriate messages
        """ 
        if(self.rex.plan_status == 1):
            self.ui.rdoutStatus.setText("Playing Back - Waypoint %d"
                                    %(self.rex.wpt_number + 1))
	self.ui.rdoutStatus.setText(self.statemachine.getmestatus(combined=True))
        


    def sliderChange(self):
        """ 
        Function to change the slider labels when sliders are moved
        and to command the arm to the given position
        """
        self.ui.rdoutBase.setText(str(self.ui.sldrBase.value()))
        self.ui.rdoutTorq.setText(str(self.ui.sldrMaxTorque.value()) + "%")
        self.rex.torque_multiplier = self.ui.sldrMaxTorque.value()/100.0
        self.rex.speed_multiplier = [self.ui.sldrSpeed.value()/100.0]*self.rex.num_joints
        self.rex.joint_angles[0] = self.ui.sldrBase.value()*D2R
        self.rex.joint_angles[1] = self.ui.sldrShoulder.value()*D2R
        self.rex.joint_angles[2] = self.ui.sldrElbow.value()*D2R
        self.rex.joint_angles[3] = self.ui.sldrWrist.value()*D2R
	if(self.rex.num_joints==6):	
		self.rex.joint_angles[4] = self.ui.sldrWrist.value()*D2R
		self.rex.joint_angles[5] = self.ui.sldrWrist.value()*D2R
		      
        self.rex.cmd_publish()

    def sliderinitChange(self): #Giri  
        """ 
        Function to change the slider labels when sliders are moved
        and to command the arm to the given position
        """
        self.ui.rdoutBase.setText(str(self.ui.sldrBase.value()))
	self.ui.rdoutShoulder.setText(str(self.ui.sldrShoulder.value() )) #Giri
        self.ui.rdoutElbow.setText(str(self.ui.sldrElbow.value() )) #Giri
        self.ui.rdoutWrist.setText(str(self.ui.sldrWrist.value() )) #Giri
        self.ui.rdoutGrip1.setText(str(self.ui.sldrGrip1.value() )) #Giri
        self.ui.rdoutGrip2.setText(str(self.ui.sldrGrip2.value() )) #Giri
        
        self.ui.rdoutTorq.setText(str(self.ui.sldrMaxTorque.value()) + "%")
	self.ui.rdoutSpeed.setText(str(self.ui.sldrSpeed.value()) + "%")	        
        
	self.rex.torque_multiplier = self.ui.sldrMaxTorque.value()/100.0
        self.rex.speed_multiplier = [self.ui.sldrSpeed.value()/100.0]*self.rex.num_joints
        #Giri{
	self.rex.joint_angles[0] = self.ui.sldrBase.value()*D2R  
        self.rex.joint_angles[1] = self.ui.sldrShoulder.value()*D2R
        self.rex.joint_angles[2] = self.ui.sldrElbow.value()*D2R
        self.rex.joint_angles[3] = self.ui.sldrWrist.value()*D2R
	if(len(self.rex.joint_angles) == 6):	
		self.rex.joint_angles[4] = self.ui.sldrGrip1.value()*D2R
        	self.rex.joint_angles[5] = self.ui.sldrGrip2.value()*D2R
        #Giri}        
	self.rex.cmd_publish()
	
    #From Giri: I implemented all these function in a one function above, incase if the response time is bad, we can go back to the below five separate functions

    def sliderOtherChange(self): #Giri
	self.ui.rdoutTorq.setText(str(self.ui.sldrMaxTorque.value()) + "%")
	self.ui.rdoutSpeed.setText(str(self.ui.sldrSpeed.value()) + "%")	        
	self.rex.torque_multiplier = self.ui.sldrMaxTorque.value()/100.0
        self.rex.speed_multiplier = [self.ui.sldrSpeed.value()/100.0]*self.rex.num_joints
        self.rex.cmd_publish()
	    
    def sliderBaseChange(self): #Kevin
        self.ui.rdoutBase.setText(str(self.ui.sldrBase.value()))
        self.rex.joint_angles[0] = self.ui.sldrBase.value()*D2R
        self.rex.cmd_publish()

    def sliderShoulderChange(self): #Kevin
        self.ui.rdoutShoulder.setText(str(self.ui.sldrShoulder.value() ))
        self.rex.joint_angles[1] = self.ui.sldrShoulder.value()*D2R
        self.rex.cmd_publish()

    def sliderElbowChange(self): #Kevin
        self.ui.rdoutElbow.setText(str(self.ui.sldrElbow.value() ))
        self.rex.joint_angles[2]=self.ui.sldrElbow.value()*D2R
        self.rex.cmd_publish()

    def sliderWristChange(self): #Kevin
        self.ui.rdoutWrist.setText(str(self.ui.sldrWrist.value() ))
        self.rex.joint_angles[3]=self.ui.sldrWrist.value()*D2R
        self.rex.cmd_publish()


    def sliderGrip1Change(self): #Giri
        self.ui.rdoutGrip1.setText(str(self.ui.sldrGrip1.value() ))
        self.rex.joint_angles[4]=self.ui.sldrGrip1.value()*D2R
        self.rex.cmd_publish()

    def sliderGrip2Change(self): #Giri
        self.ui.rdoutGrip2.setText(str(self.ui.sldrGrip2.value() ))
        self.rex.joint_angles[5]=self.ui.sldrGrip2.value()*D2R
        self.rex.cmd_publish()
    


    def mousePressEvent(self, QMouseEvent):
        """ 
        Function used to record mouse click positions for calibration 
        """
        """ Get mouse posiiton """
        x = QMouseEvent.x()
        y = QMouseEvent.y()

        """ If mouse position is not over the camera image ignore """
        if ((x < MIN_X) or (x > MAX_X) or (y < MIN_Y) or (y > MAX_Y)): return

        """ Change coordinates to image axis """
        self.last_click[0] = x - MIN_X
        self.last_click[1] = y - MIN_Y

        """ If calibration has been performed """
        if (self.video.cal_flag == 1):
            
            """ Save last mouse coordinate """
            self.video.mouse_coord[self.video.mouse_click_id] = [(x-MIN_X),(y-MIN_Y)]

            """ Update the number of used poitns for calibration """
            self.video.mouse_click_id += 1
            """ Update status label text """
            self.ui.rdoutStatus.setText("Affine Calibration: Click Point %d is recorded, please select Click Point %d"
                                      %(self.video.mouse_click_id, self.video.mouse_click_id+1))

            """ 
            If the number of click is equal to the expected number of points
            computes the affine calibration.
            
            LAB TASK: Change this code to use your workspace calibration routine
            and NOT the simple calibration function as is done now.
            """
            if(self.video.mouse_click_id == self.video.cal_points):
                """ 
                Update status of calibration flag and number of mouse
                clicks
                """
                self.video.cal_flag = 2
                self.video.mouse_click_id = 0
                
                """ Perform affine calibration with OpenCV """
                self.video.aff_matrix = cv2.getAffineTransform(
                                        self.video.mouse_coord,
                                        self.video.real_coord)
            	#print self.video.mouse_coord #Josh
		#print self.video.real_coord #Josh
                """ Updates Status Label to inform calibration is done """ 
                self.ui.rdoutStatus.setText("Waiting for input")
	if (self.video.dep2rgb_aff_flag == 1): #Josh
	    if (self.video.mouse_click_id < self.video.rgb_pts_num):
		self.video.rgb_coord[self.video.mouse_click_id*2] = x-MIN_X
		self.video.rgb_coord[self.video.mouse_click_id*2+1] = y-MIN_Y
	    else:
		self.video.dep_coord[2*(self.video.mouse_click_id - self.video.rgb_pts_num)] = [(x-MIN_X),(y-MIN_Y),1,0,0,0]
		self.video.dep_coord[2*(self.video.mouse_click_id - self.video.rgb_pts_num)+1] = [0,0,0, (x-MIN_X),(y-MIN_Y),1]
	    self.video.mouse_click_id += 1
	    print self.video.mouse_click_id
	    if (self.video.mouse_click_id == self.video.dep_pts_num):
		self.video.dep2rgb_aff_flag = 2
		self.video.mouse_click_id = 0
		A = self.video.dep_coord
		A_trans = np.transpose(A)
		A_AT = np.dot(A_trans, A)
		A_AT_inv = np.linalg.inv(A_AT)
		A_all = np.dot(A_AT_inv,A_trans)
		b = np.transpose(self.video.rgb_coord)
		tmp_dep2rgb_aff_array = np.dot(A_all, b)
		upper_dep2rgb_aff_matrix = np.reshape(tmp_dep2rgb_aff_array,(2, 3))
		lower_dep2rgb_aff_matrix = np.array([[0,0,1]])
		self.video.dep2rgb_aff_matrix = np.concatenate((upper_dep2rgb_aff_matrix,lower_dep2rgb_aff_matrix), axis=0)
		self.video.rgb2dep_aff_matrix = np.linalg.inv(self.video.dep2rgb_aff_matrix)
		print self.video.rgb2dep_aff_matrix
	if (self.video.extrinsic_cal_flag == 1): #Josh
	    self.video.extrinsic_cal_pixel_coord[self.video.mouse_click_id] = [(x-MIN_X),(y-MIN_Y)]
	    self.video.mouse_click_id += 1
	    if (self.video.mouse_click_id == 3):
		"""
		retval, rvec, tvec = cv2.solvePnP(self.video.extrinsic_cal_world_coord, self.video.extrinsic_cal_pixel_coord, self.video.intrinsic, self.video.distortion_array)
		#print self.video.extrinsic_cal_pixel_coord, self.video.extrinsic_cal_world_coord		
		self.video.mouse_click_id = 0
		R,tmp = cv2.Rodrigues(rvec)
		tmp = np.concatenate((np.transpose(R),-1.0*tvec),axis=1)
		self.video.extrinsic = np.concatenate((tmp, np.float32([[0., 0., 0., 1.]])),axis = 0)
		print self.video.extrinsic
		self.video.inv_extrinsic = np.linalg.inv(self.video.extrinsic)
		self.video.extrinsic_cal_flag = 2
		"""
		self.video.mouse_click_id = 0
		tmp_coord = np.float32([[self.video.extrinsic_cal_pixel_coord[0]],[self.video.extrinsic_cal_pixel_coord[1]],[self.video.extrinsic_cal_pixel_coord[2]]])
		camera_coord = 941.0*cv2.undistortPoints(tmp_coord,self.video.intrinsic,self.video.distortion_array)
		rot_rad = -1.0*np.arctan2((camera_coord[1][0][1]-camera_coord[0][0][1]),(camera_coord[1][0][0]-camera_coord[0][0][0]))
		print "rotation in degrees = ", rot_rad*R2D
		if rot_rad < 0:
		    z_rot = rot_rad + 0.5*np.pi
		else:
		    z_rot = 0.5*np.pi-rot_rad
		R_T = np.array([[np.cos(z_rot),np.sin(z_rot),0.0],[np.sin(z_rot),-1.0*np.cos(z_rot),0.0],[0.0,0.0,-1.0]])
		R = np.transpose(R_T)
		T = np.float32([(camera_coord[0][0][0]+camera_coord[2][0][0])/2,(camera_coord[0][0][1]+camera_coord[2][0][1])/2,941])
		T = np.transpose(np.array([-1.0*np.dot(R,T)]))
		tmp = np.concatenate((R,T),axis=1)
		self.video.extrinsic = np.concatenate((tmp, np.float32([[0., 0., 0., 1.]])),axis = 0)
		#self.video.writecaldata(self.video.extrinsic)
		self.video.inv_extrinsic = np.linalg.inv(self.video.extrinsic)
		print "Extrinsic Matrix = ", self.video.extrinsic
		self.video.extrinsic_cal_flag = 2



    def deprgb_cali(self): #Josh
	self.video.dep2rgb_aff_flag = 1
	self.ui.rdoutStatus.setText("Start camera calibration by clicking mouse to select Click Point 1")
    

    def extrinsic_cali(self): #Josh

	alpha = 525.91386088
	u0 = 312.41480582
	beta = 526.52064559
	v0 =247.78962122
	matrix1 = np.float32([[1/alpha, 0., 0.], [0.,1/beta, 0.], [0.,0.,1.0]])
	matrix2 = np.float32([[1.0, 0., -u0], [0.,1., -v0], [0.,0.,1.0]])
	self.video.inv_intrinsic = np.dot(matrix1,matrix2)
	self.video.intrinsic = np.float32([[alpha, 0., u0],[0., beta, v0],[0., 0., 1.]])
	self.video.distortion_array = np.float32([2.45479098e-01,-8.27672136e-01, 1.05870966e-03, -3.01947277e-03,1.08683931e+00])
	self.video.extrinsic_cal_flag = 1
	  


    def teach(self): #Giri
        self.statemachine.teach(self.ui,self.rex) #Giri
        	
    def repeat(self): #Giri
	if(self.ui.btnUser7.text() == "Repeat"):
		self.statemachine.init(self.ui,self.rex,"Initialising")	
	else:
		self.statemachine.repeat(self.ui,self.rex, False) #Giri
	
    def srepeat(self): #Giri
	self.statemachine.init(self.ui,self.rex,"initialising")

    def reset(self, hold = False):
	self.rex.torque_multiplier =self.ui.sldrMaxTorque.value()/100.0
        self.rex.speed_multiplier = [self.ui.sldrSpeed.value()/100.0]*self.rex.num_joints
        
	self.rex.joint_angles[0] = 0.0
        self.rex.joint_angles[1] = 0.0
        self.rex.joint_angles[2] = 0.0
        self.rex.joint_angles[3] = 0.0
	if(len(self.rex.joint_angles) == 6 and hold==False):	
		self.rex.joint_angles[4] = 0.0
        	self.rex.joint_angles[5] = 95*D2R
     	elif(len(self.rex.joint_angles) == 6 and hold==True):	
		self.rex.joint_angles[4] = 0.0
        	self.rex.joint_angles[5] = -95*D2R
     
	self.rex.cmd_publish()


    def getheight(self,angle,action = "picking"):
	placement_offset = 2 # in cm
	if(angle-90<0.1):
		gripping_height = 3
	else:
		if(action == "picking"):
			gripping_height = 0.05
		elif(action == "placing"):
			gripping_height = 0.05 + placement_offset
	return gripping_height

    def getIK(self,endCoord,angle=0,action = "picking"):
	intermediate_height = 4
	intermediate_angles = [0.0]*6
	angles = [0.0]*6
	[angles[0],angles[1],angles[2],angles[3]] = inverseKinematics(endCoord[0],endCoord[1],endCoord[2]+self.getheight(endCoord[3],action),endCoord[3])
	if(angles[0] == 0 and angles[1] == 0 and angles[2] == 0 and angles[3] == 0):
		if(abs(endCoord[3]-90)<0.1):
			endCoord[3] = 180
			[angles[0],angles[1],angles[2],angles[3]] = inverseKinematics(endCoord[0],endCoord[1],endCoord[2]+self.getheight(endCoord[3],action),endCoord[3])
		else:
			endCoord[3] = 90
			[angles[0],angles[1],angles[2],angles[3]] = inverseKinematics(endCoord[0],endCoord[1],endCoord[2]+self.getheight(endCoord[3],action),endCoord[3])
	
	if(endCoord[3]==180):
		[intermediate_angles[0],intermediate_angles[1],intermediate_angles[2],intermediate_angles[3]] = inverseKinematics(endCoord[0],endCoord[1],endCoord[2]+self.getheight(endCoord[3],action)+intermediate_height,endCoord[3]) # assuming it is reachable	
		print self.trim_angle(orientation(angles[0],angle*R2D))		
		if(action == "placing"):
			angles[4] = 90 + angles[0]
		else:
			angles[4] = -1*self.trim_angle(orientation(angles[0],angle*R2D))
		
	else:		
		angles[4] = 0

	intermediate_angles[4] = angles[4]
	angles[5] = endCoord[3]
	intermediate_angles[5] = endCoord[3]
	return [angles,intermediate_angles]

    def printfk(self,angles):
	temp = forwardKinematics(angles[0],angles[1],angles[2],angles[3])
	print "Forward Kinematics output =  " , str(temp[0]) ,str(temp[1]),str(temp[2]),str(temp[3])	
	
    def roundoff(self, angles):
	angles[0] = round(self.trim_angle(angles[0]),2)
	angles[1] = round(self.trim_angle(angles[1]),2)	
	angles[2] = round(self.trim_angle(angles[2]),2)
	angles[3] = round(self.trim_angle(angles[3]),2)
	angles[4] = round(self.trim_angle(angles[4]),2) #Dont alter these two angles
	angles[5] = round(self.trim_angle(angles[5]),2)
	return angles


    def pick(self): #Josh
	global putx, puty, putz, putangle
	blockx, blocky, blockz, angle = self.get_color_block_world_coord('blue')
	#self.statemachine.setorientation(angle*R2D)
	[putx, puty, putz, putangle] = [-1*blockx, blocky, blockz, angle]
	#endCoord = [20.2, -16.1, 4.4,90]
	#endCoord = [14.3, 19.4, 4.4,90]
	endCoord = [blockx/10, blocky/10, (blockz)/10, gripper_orientation]		
	[angles, intermediate_angles] = self.getIK(endCoord,angle)
	if(debug):
		print "Block detector output = ", [blockx, blocky, blockz, angle*R2D]
		print "Input to Inverse Kinematics:" ,endCoord
		print "Output of Inverse Kinematics: ",angles		
		self.printfk(angles)
		
	angles = self.roundoff(angles)
	intermediate_angles = self.roundoff(intermediate_angles)
	self.statemachine.setq(angles,intermediate_angles)
	self.statemachine.setmystatus("testing", "picking","picking")#mode="testing",action="picking")
	#self.statemachine.hold(self.ui,self.rex, angles,"picking")
    def place(self):

	endCoord = [putx/10, puty/10, (putz)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,putangle)
	
	#wangle = (self.shortorientation((putangle)*R2D-45)-self.trim_angle(angles[0]))#self.shortorientation((angle)*R2D+45)
	#self.statemachine.set_orientations(wangle,2)
	if(debug):
		print "Placing according to Block detector output = ", [putx, puty, putz, putangle*R2D]
		print "Input to Inverse Kinematics:" ,endCoord
		self.printfk(angles)
	
	angles = self.roundoff(angles)
	intermediate_angles = self.roundoff(intermediate_angles)

	self.statemachine.setq(angles,intermediate_angles)
	self.statemachine.setmystatus("testing", "placing","placing") #mode="testing",action="placing")	
#	self.statemachine.hold(self.ui,self.rex, angles,"placing")


    def get_color_block_world_coord(self,color): #Josh
	self.video.blockDetector()
	[blockx_rgb, blocky_rgb, angle] = self.video.color_detection(color)
	if(blockx_rgb==0 and blocky_rgb==0 and angle==0):
		return 0,0,0,0
	xy_in_rgb = np.float32([[[blocky_rgb,blockx_rgb]]]) #Josh
	xy_in_rgb_homogenous = np.array([blocky_rgb,blockx_rgb,1.0])
	xy_in_dep = np.dot(self.video.rgb2dep_aff_matrix, xy_in_rgb_homogenous) #Josh
	x_dep = int(xy_in_dep[0]) #Josh
	y_dep = int(xy_in_dep[1]) #Josh
        d = self.video.currentDepthFrame[y_dep][x_dep]
	Z = -1
	for idx in range(len(self.video.levels_mm)):
	    if d <=self.video.levels_upper_d[idx] and d>=self.video.levels_lower_d[idx]:
		Z = 941 - self.video.levels_mm[idx]
		break
	camera_coord = Z*cv2.undistortPoints(xy_in_rgb,self.video.intrinsic,self.video.distortion_array)
	camera_coord_homogenous = np.transpose(np.append(camera_coord,[Z,1.0]))
	world_coord_homogenous = np.dot(self.video.extrinsic, camera_coord_homogenous)
	theta = np.arccos(self.video.extrinsic[0,0])
	angle = angle - theta
	return world_coord_homogenous[0], world_coord_homogenous[1], world_coord_homogenous[2], angle

    def getangles(self, color):
	blockx, blocky, blockz, angle = self.get_color_block_world_coord(color)
	if(blockx == 0 and blocky == 0 and blockx == 0 and angle == 0):
		angles = [0]*6
		intermediate_angles = [0]*6
	else:		
		[current_mode,current_action, current_movement, current_motorstate]=self.statemachine.getmestatus()
		endCoord = [(blockx)/10, (blocky)/10, (blockz)/10, gripper_orientation]	
		
		[angles,intermediate_angles] = self.getIK(endCoord,angle,"picking")
	
		angles = self.roundoff(angles)
		intermediate_angles = self.roundoff(intermediate_angles)
	self.statemachine.setq(angles, intermediate_angles)

    def generatecomp2(self):
	print "Entered generatecomp2"
	new_q = np.zeros([3,6])
	new_qh = np.zeros([3,6]) #intermediate heights
	blockx, blocky, blockz, angle = self.get_color_block_world_coord('blue')
	final_x = blockx
	final_y = blocky	
	height = 40
	endCoord = [-(final_x)/10, (final_y)/10, (blockz+ height*0)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
		
	new_q[0] = self.roundoff(angles)
	new_qh[0] = self.roundoff(intermediate_angles)
	
	endCoord = [-(final_x)/10, (final_y)/10, (blockz+height*1)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	new_q[1] = self.roundoff(angles)
	new_qh[1] = self.roundoff(intermediate_angles)
	
	endCoord = [-(final_x)/10, (final_y)/10, (blockz+height*2)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	new_q[2] = self.roundoff(angles)
	new_qh[2] = self.roundoff(intermediate_angles)
	
	self.statemachine.setq_comp(new_q,new_qh)
 
    def generatecomp3(self):
	print "Entered generatecomp3"
	new_q = np.zeros([8,6])
	new_qh = np.zeros([8,6]) #intermediate heights
	blockx, blocky, blockz, angle = self.get_color_block_world_coord('blue')
	final_x = -140
	final_y = -89.5	
	b = 40
	endCoord = [(final_x+b*0)/10, (final_y)/10, (blockz)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
		
	new_q[0] = self.roundoff(angles)
	new_qh[0] = self.roundoff(intermediate_angles)
	
	endCoord = [(final_x+b*1)/10, (final_y)/10, (blockz)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	new_q[1] = self.roundoff(angles)
	new_qh[1] = self.roundoff(intermediate_angles)
	print "Genereated FK: ", forwardKinematics(new_q[1][0],new_q[1][1],new_q[1][2],new_q[1][3])
			
	endCoord = [(final_x+b*2)/10, (final_y)/10, (blockz)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	new_q[2] = self.roundoff(angles)
	new_qh[2] = self.roundoff(intermediate_angles)
	
	endCoord = [(final_x+b*3)/10, (final_y)/10, (blockz)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
		
	new_q[3] = self.roundoff(angles)
	new_qh[3] = self.roundoff(intermediate_angles)
	
	endCoord = [(final_x+b*4)/10, (final_y)/10, (blockz)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	new_q[4] = self.roundoff(angles)
	new_qh[4] = self.roundoff(intermediate_angles)
	
	endCoord = [(final_x+b*5)/10, (final_y)/10, (blockz)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	new_q[5] = self.roundoff(angles)
	new_qh[5] = self.roundoff(intermediate_angles)
	
	endCoord = [(final_x+b*6)/10, (final_y)/10, (blockz)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
		
	new_q[6] = self.roundoff(angles)
	new_qh[6] = self.roundoff(intermediate_angles)
	
	endCoord = [(final_x+b*7)/10, (final_y)/10, (blockz)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	new_q[7] = self.roundoff(angles)
	new_qh[7] = self.roundoff(intermediate_angles)	

	self.statemachine.setq_comp(new_q,new_qh)
	

    def generatecomp4(self):
	print "Entered generatecomp4"
	new_q = np.zeros([8,6])
	new_qh = np.zeros([8,6]) #intermediate heights
	blockx, blocky, blockz, angle = self.get_color_block_world_coord('blue')
	final_x = blockx
	final_y = blocky	
	height = 40
	endCoord = [-(final_x)/10, (final_y)/10, (blockz+ height*0)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
		
	new_q[0] = self.roundoff(angles)
	new_qh[0] = self.roundoff(intermediate_angles)
	
	endCoord = [-(final_x)/10, (final_y)/10, (blockz+height*1)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	new_q[1] = self.roundoff(angles)
	new_qh[1] = self.roundoff(intermediate_angles)
	print "Genereated FK: ", forwardKinematics(new_q[1][0],new_q[1][1],new_q[1][2],new_q[1][3])
			
	endCoord = [-(final_x)/10, (final_y)/10, (blockz+height*2)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	new_q[2] = self.roundoff(angles)
	new_qh[2] = self.roundoff(intermediate_angles)
	
	endCoord = [-(final_x)/10, (final_y)/10, (blockz+ height*3)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
		
	new_q[3] = self.roundoff(angles)
	new_qh[3] = self.roundoff(intermediate_angles)
	
	endCoord = [-(final_x)/10, (final_y)/10, (blockz+height*4)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	new_q[4] = self.roundoff(angles)
	new_qh[4] = self.roundoff(intermediate_angles)
	
	endCoord = [-(final_x)/10, (final_y)/10, (blockz+height*5)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	new_q[5] = self.roundoff(angles)
	new_qh[5] = self.roundoff(intermediate_angles)
	
	endCoord = [-(final_x)/10, (final_y)/10, (blockz+ height*6)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	# hard code joints angles down below here	
	new_q[6] = self.roundoff(angles)
	new_qh[6] = self.roundoff(intermediate_angles)
	
	endCoord = [-(final_x)/10, (final_y)/10, (blockz+height*7)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	new_q[7] = self.roundoff(angles)
	new_qh[7] = self.roundoff(intermediate_angles)	

	self.statemachine.setq_comp(new_q,new_qh)
	
    def generatecomp5(self):
	print "Entered generatecomp5"
	new_q = np.zeros([8,6])
	new_qh = np.zeros([8,6]) #intermediate heights
	blockx, blocky, blockz, angle = self.get_color_block_world_coord('blue')
	final_x = -140
	final_y = -89.5	
	b = 40
	endCoord = [(final_x+b*0)/10, (final_y)/10, (blockz)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
		
	new_q[0] = self.roundoff(angles)
	new_qh[0] = self.roundoff(intermediate_angles)
	
	endCoord = [(final_x+b*1)/10, (final_y)/10, (blockz)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	new_q[1] = self.roundoff(angles)
	new_qh[1] = self.roundoff(intermediate_angles)
	print "Genereated FK: ", forwardKinematics(new_q[1][0],new_q[1][1],new_q[1][2],new_q[1][3])
			
	endCoord = [(final_x+b*2)/10, (final_y)/10, (blockz)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	new_q[2] = self.roundoff(angles)
	new_qh[2] = self.roundoff(intermediate_angles)
	
	endCoord = [(final_x+b*3)/10, (final_y)/10, (blockz)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
		
	new_q[3] = self.roundoff(angles)
	new_qh[3] = self.roundoff(intermediate_angles)
	
	endCoord = [(final_x+b*4)/10, (final_y)/10, (blockz)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	new_q[4] = self.roundoff(angles)
	new_qh[4] = self.roundoff(intermediate_angles)
	
	endCoord = [(final_x+b*5)/10, (final_y)/10, (blockz)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	new_q[5] = self.roundoff(angles)
	new_qh[5] = self.roundoff(intermediate_angles)
	
	endCoord = [(final_x+b*6)/10, (final_y)/10, (blockz)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
		
	new_q[6] = self.roundoff(angles)
	new_qh[6] = self.roundoff(intermediate_angles)
	
	endCoord = [(final_x+b*7)/10, (final_y)/10, (blockz)/10, gripper_orientation]	
	[angles,intermediate_angles] = self.getIK(endCoord,0,"placing")
	new_q[7] = self.roundoff(angles)
	new_qh[7] = self.roundoff(intermediate_angles)	

	self.statemachine.setq_comp(new_q,new_qh)
	

    def competition(self):
	if(self.ui.btnUser11.text() == "Enter Competition Mode"):
		self.ui.btnUser11.setText("Enter Testing Mode")
		self.ui.btnUser7.setText("Draw")
		self.ui.btnUser1.setText("Competition 1")
		self.ui.btnUser2.setText("Competition 2")
		self.ui.btnUser3.setText("Competition 3")
		self.ui.btnUser4.setText("Competition 4")
		self.ui.btnUser5.setText("Competition 5")
       		self.ui.btnUser2.setEnabled(True) 	
		self.ui.btnUser7.setEnabled(True) 	
		

	elif(self.ui.btnUser11.text() == "Enter Testing Mode"):
		self.ui.btnUser11.setText("Enter Competition Mode")
        	self.ui.btnUser7.setText("Smooth Repeat")
		self.ui.btnUser1.setText("Configure Servos")
		self.ui.btnUser2.setText("Depth and RGB Calibration")
		self.ui.btnUser3.setText("Extrinsic Calibration")
		self.ui.btnUser4.setText("Block Detection")
		self.ui.btnUser5.setText("Repeat")
		self.ui.btnUser2.setEnabled(False) 	
		self.ui.btnUser7.setEnabled(False) 	


    def btn1(self): 
	if(self.ui.btnUser1.text()=="Configure Servos"):
        	self.rex.cfg_publish_default()
	elif(self.ui.btnUser1.text()=="Competition 1"):
		if(self.sanity_check(1)):
			self.getangles('blue')		
			self.statemachine.setmystatus("Competition 1", "picking","picking")#mode="testing",action="picking")	
		
    def btn2(self): 
	if(self.ui.btnUser2.text()=="Depth and RGB Calibration"):
        	self.deprgb_cali()
	elif(self.ui.btnUser2.text()=="Competition 2"):
		if(self.sanity_check(2)):
			self.generatecomp2()
			self.getangles('blue')		
			self.statemachine.setmystatus("Competition 2", "picking","picking")#mode="testing",action="picking")	
		
    def btn3(self): 
	if(self.ui.btnUser3.text()=="Extrinsic Calibration"):
        	self.extrinsic_cali()
	elif(self.ui.btnUser3.text()=="Competition 3"):
		if(self.sanity_check(3)):
			self.generatecomp3()
			self.getangles('black')		
			self.statemachine.setmystatus("Competition 3", "picking","picking")#mode="testing",action="picking")	
	
    def btn4(self): 
	if(self.ui.btnUser4.text()=="Block Detector"):
        	self.video.blockDetector()
	elif(self.ui.btnUser4.text()=="Competition 4"):
		if(self.sanity_check(4)):
			self.generatecomp4()
			self.getangles('black')		
			self.statemachine.setmystatus("Competition 4", "picking","picking")#mode="testing",action="picking")	
	
    def btn5(self): 
	if(self.ui.btnUser5.text()=="Repeat"):
        	self.repeat()
	elif(self.ui.btnUser5.text()=="Competition 5"):
		self.generatecomp5()
		self.getangles('all')		
		self.statemachine.setmystatus("Competition 5", "picking","picking")#mode="testing",action="picking")	


    def sanity_check(self,comp):
	success = False	
	if(comp == 1 or comp == 2): #search for red,blue,green 
		colors = ["red","blue","green"]
		[pos_colors, success] = self.video.blockDetector(colors)
	elif(comp == 4 or comp == 3): #search for red,blue,green 
		colors = ["red","blue","green","orange","violet","black","yellow","pink"]
		[pos_colors, success] = self.video.blockDetector(colors)
		
	return success
	
			
    """main function"""
def main():
    app = QtGui.QApplication(sys.argv)
    app_window = Gui()
    app_window.show()
    sys.exit(app.exec_())
 
if __name__ == '__main__':
    main()
