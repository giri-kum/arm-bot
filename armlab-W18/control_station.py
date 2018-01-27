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

""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592

""" Pyxel Positions of image in GUI """
MIN_X = 310
MAX_X = 950

MIN_Y = 30
MAX_Y = 510
 
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

        """ Other Variables """
        self.last_click = np.float32([0,0])

        """ Set GUI to track mouse """
        QtGui.QWidget.setMouseTracking(self,True)

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
	self.ui.sldrGrip1.setEnabled(True) #Giri
	self.ui.sldrGrip2.setEnabled(True) #Giri
	
        """ Initial command of the arm to home position"""
        self.sliderChange() 
        
        """ Connect Buttons to Functions 
        TODO: NAME AND CONNECT BUTTONS AS NEEDED
        """
        self.ui.btnUser1.setText("Configure Servos")
        self.ui.btnUser1.clicked.connect(self.rex.cfg_publish_default)
        self.ui.btnUser12.setText("Emergency Stop!")
        self.ui.btnUser12.clicked.connect(self.estop)


	self.ui.btnUser2.setText("Depth and RGB Calibration")
	self.ui.btnUser2.clicked.connect(self.deprgb_cali) #Josh
        self.ui.btnUser3.setText("Extrinsic Calibration") #Josh
	self.ui.btnUser3.clicked.connect(self.extrinsic_cali)
	self.ui.btnUser4.setText("Block Detector") #Josh
	self.ui.btnUser4.clicked.connect(self.video.blockDetector)
	self.ui.btnUser5.setText("Pick Block")
	self.ui.btnUser5.clicked.connect(self.pick)
	self.ui.btnUser6.setText("Place Block")
	self.ui.btnUser6.clicked.connect(self.place)

	self.ui.btnUser7.setText("Draw")
	self.ui.btnUser7.clicked.connect(self.teach) #Giri
	self.ui.btnUser8.setText("Enter Competition")
	self.ui.btnUser8.clicked.connect(self.teach) #Giri


		
	self.ui.btnUser9.setText("Teach")
	self.ui.btnUser9.clicked.connect(self.teach) #Giri
        self.ui.btnUser10.setText("Repeat")
	self.ui.btnUser10.clicked.connect(self.repeat) #Giri
	self.ui.btnUser11.setText("Smooth Repeat")
	self.ui.btnUser11.clicked.connect(self.srepeat) #Giri        
	self.ui.btnUser11.setEnabled(False) #Giri			
	

    def estop(self):
        self.statemachine.estop(self.rex)

    def trim_angle(self,q):
	if(q>360 or q<-360):
		new_q = 0
	elif(q>180): #181 --> -179
		new_q = q-360
	elif(q<-180):
		new_q = 360-q
	else :
		new_q = q
	return new_q


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
	
	angles = inverseKinematics(endCoord[0],endCoord[1],endCoord[2],endCoord[3])
	angles[0] = round(self.trim_angle(angles[0]),2)
	angles[1] = round(self.trim_angle(angles[1]),2)	
	angles[2] = round(self.trim_angle(angles[2]),2)
	angles[3] = round(self.trim_angle(angles[3]),2)
	#print angles
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
		print rot_rad
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
		self.video.inv_extrinsic = np.linalg.inv(self.video.extrinsic)
		print self.video.extrinsic
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

    def pick(self): #Josh
	blockx, blocky, blockz, angle = self.get_color_block_world_coord('blue')
	print [blockx, blocky, blockz, angle]
	endCoord = [blockx/10.0, blocky/10.0, (blockz+30)/10.0, 90]
	angles = inverseKinematics(endCoord[0],endCoord[1],endCoord[2],endCoord[3])
	print "endcoord:"
	print endCoord
	angles[0] = round(self.trim_angle(angles[0]),2)
	angles[1] = round(self.trim_angle(angles[1]),2)	
	angles[2] = round(self.trim_angle(angles[2]),2)
	angles[3] = round(self.trim_angle(angles[3]),2)
	
 	self.rex.joint_angles[0] = angles[0]*D2R
        self.rex.joint_angles[1] = angles[1]*D2R
        self.rex.joint_angles[2] = angles[2]*D2R
        self.rex.joint_angles[3] = angles[3]*D2R
 	self.rex.cmd_publish()
    	
 
    def place(self):
	pass
    
    def get_color_block_world_coord(self,color): #Josh
	self.video.blockDetector()
	[blockx_rgb, blocky_rgb], angle = self.video.color_detection(color)
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
	return world_coord_homogenous[0], world_coord_homogenous[1], world_coord_homogenous[2], angle


    def teach(self): #Giri
        self.statemachine.teach(self.ui,self.rex) #Giri
        	
    def repeat(self): #Giri
	if(self.ui.btnUser5.text() == "Repeat"):
		self.statemachine.init(self.ui,self.rex,"Initialising")	
	else:
		self.statemachine.repeat(self.ui,self.rex, False) #Giri
	
	
    def srepeat(self): #Giri
	self.statemachine.init(self.ui,self.rex,"initialising")
    	
"""main function"""
def main():
    app = QtGui.QApplication(sys.argv)
    app_window = Gui()
    app_window.show()
    sys.exit(app.exec_())
 
if __name__ == '__main__':
    main()
