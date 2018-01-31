import cv2
import numpy as np
from PyQt4 import QtGui, QtCore, Qt
import freenect
import matplotlib.pyplot as plt

class Video():
    def __init__(self):
        self.currentVideoFrame=np.array([])
        self.currentDepthFrame=np.array([])
        self.kinectConnected = 1 #expect kinect to be available

        """ Extra arrays for colormaping the depth image"""
        self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthCM=np.array([])

        
        """ 
        Calibration Variables 
        
        Currently this takes three points to perform a simple calibration.
        The simple calibration only worls for points on the board and uses
        an affine transform to perform the calibration.

        To use: 
            1. Click on simple calibration button
            2. Click the center of the arm
            3. Click the upper right corner of the board
            4. Click the lower right corner of the board

        Note that OpenCV requires float32 arrays

        TODO: Modify these to enable a robust 3D calibration

        """
        self.cal_points = 3 # number of points for the simple calibration
        self.real_coord = np.float32([[0., 0.], [305.,-305.], [-305.,-305.]])
        self.mouse_coord = np.float32([[0.0, 0.0],[0.0, 0.0],[0.0, 0.0]])
        self.rgb_coord = np.float32([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) #Josh
	self.dep_coord = np.empty((6,6)) # Josh     
        self.mouse_click_id = 0
        self.cal_flag = 0 # 0 - not calibrated, 1 - in progress, 2 - done
        self.aff_matrix = np.empty((2,3)) # affine matrix for simple calibration
	self.rgb2dep_aff_matrix = np.array([[1.09403672e0, 3.44369111e-3, -1.62867595e0],[-2.41966785e-3, 1.07534829e0,-2.69041712e1],[0., 0., 1.]]) #Josh #rgb_coord = dep2rgb_aff_matrix * dep_cord
	self.dep2rgb_aff_matrix = np.linalg.inv(self.rgb2dep_aff_matrix) #Josh	
	self.dep2rgb_aff_flag = 0 #Josh
	self.dep_pts_num = 6 #Josh
	self.rgb_pts_num = 3 #Josh
	self.intrinsic_matrix = np.empty((3,3)) #Josh
	self.distortion_array = np.float32([0.0, 0.0, 0.0, 0.0, 0.0]) #Josh
	self.inv_intrinsic = np.empty((3,3)) #Josh
	self.intrinsic = np.empty((3,3)) #Josh
	self.inv_extrinsic = np.empty((4,4)) #Josh
	self.extrinsic = np.empty((4,4)) #Josh
	#self.extrinsic = np.array(self.getcaldata())
	#self.inv_extrinsic = np.array(np.linalg.inv(self.extrinsic))

	self.extrinsic_cal_flag = 0 #Josh
	self.extrinsic_cal_pixel_coord = np.float32([[0.0, 0.0],[0.0, 0.0],[0.0, 0.0]]) #Josh
	self.extrinsic_cal_world_coord = np.float32([[-305., -305., 0.],[-305., 305.,0.],[305.0, 305.0,0.]]) #Josh
 	self.levels_mm = np.float32([0., 19.25, 57.5, 96.25, 134.75, 173.25, 211.75, 250.25])
	self.levels_lower_d = np.float32([716,705,687,668,648,624,598,570,538])#np.float32([716,697,680,659,637,615,586,557,530])
	self.levels_upper_d = np.float32([726,710,692,675,653,630,604,575,543])#np.float32([726,715,696,679,658,636,614,585,556])
	self.depth_image_min_row = 0
	self.depth_image_max_row = 0
	self.depth_image_min_col = 0
	self.depth_image_max_col = 0
	self.blobs_rectangles = [] #  rectangle =((centercoord),(width,height),angle)
	self.blobs_box_pts = [] #box_points = [[four corners coords]]
	self.blobs_box_pts_rgb_frame = []
	self.blobs_centers_rgb = []
	self.blobs_angle_rgb = []
	self.blobs_info_rgb = []
	#due to light from roof, yellow shows 0,0,254 in hsv color space. red goes to two ends of hsv spectral, needs to consider seperately
	self.hsv_lower_hsv = [[0.,9.,15.],[6.,155.,140.],[0.,0.,200.],[40.,30.,75.],[110.,50.,80.],[130.,70.,60.],[164.,150.,170.]] #Josh
	self.hsv_upper_hsv = [[180.,243.,75.],[20.,255.,255.],[39.,30.,255.],[85.,200.,210.],[129.,255.,255.],[168.,190.,160.],[177.,244.,255.]] #Josh
	self.hsv_colors = ['black','orange','yellow','green','blue','violet','pink']


    def captureVideoFrame(self):
        """                      
        Capture frame from Kinect, format is 24bit RGB    
        """
        self.currentVideoFrame = freenect.sync_get_video()[0]

    def captureDepthFrame(self):
        """                      
        Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        self.currentDepthFrame = freenect.sync_get_depth()[0]

    
    def loadVideoFrame(self):
        self.currentVideoFrame = cv2.cvtColor(
            cv2.imread("data/ex0_bgr.png",cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB
            )

    def loadDepthFrame(self):
        self.currentDepthFrame = cv2.imread("data/ex0_depth16.png",0)

    def convertFrame(self):
        """ Converts frame to format suitable for QtGui  """
        try:
            img=QtGui.QImage(self.currentVideoFrame,
                             self.currentVideoFrame.shape[1],
                             self.currentVideoFrame.shape[0],
                             QtGui.QImage.Format_RGB888
                             )
            return QtGui.QPixmap.fromImage(img)
        except:
            return None

    def convertDepthFrame(self):
        """ Converts frame to a colormaped format suitable for QtGui  
            Note: this cycles the spectrum over the lowest 8 bits
        """
        try:

            """ 
            Convert Depth frame to rudimentary colormap
            """
            self.DepthHSV[...,0] = self.currentDepthFrame
            self.DepthHSV[...,1] = 0x9F
            self.DepthHSV[...,2] = 0xFF
            self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)

            img=QtGui.QImage(self.DepthCM,
                             self.DepthCM.shape[1],
                             self.DepthCM.shape[0],
                             QtGui.QImage.Format_RGB888
                             )
            return QtGui.QPixmap.fromImage(img)
        except:
            return None

    def getcaldata(self):
	file = open("excal.csv", "r")
	data = file.read();
	lines = data.splitlines();
	A = np.zeros([4,4])
	n = len(lines)
	if(n==4):
		for i in range(0,n):
			A[i] = map(float, lines[i].split(","));
		
	return A

    def writecaldata(self,A):
	data = ""
	for y in A :
		data = data + ','.join(str(z) for z in y) + "\n"
	file = open("excal.csv", "w")
	file.write(data)
	file.close()
	


    def loadCalibration(self):
        """
        TODO (OPTIONAL):
        Load csmera distortion calibration from file and applies to the image:
        Look at camera_cal.py final lines to see how that is done
        This is optional, you do not need to implement distortion correction
        """
        pass
    
    def blockDetector(self): #Josh
        """
        TODO:
        Implement your block detector here.  
        You will need to locate
        blocks in 3D space
        """
	self.blobs_rectangles = []
	self.blobs_box_pts = []
        self.captureDepthFrame()
	depth_image = self.currentDepthFrame
	(dep_rows,dep_cols) = np.shape(depth_image)
	min_row = 480
	min_col = 640
	max_row = 0
	max_col = 100
 	#get the rough range of board, 727 is the boundary of depth reading of board's surface and surfaces under it.
	for cur_row in range(dep_rows-5):
	    for cur_col in range(dep_cols-5):
		if depth_image[cur_row][cur_col] < 727 and depth_image[cur_row+5][cur_col] < 727 and depth_image[cur_row][cur_col+5] < 727 and depth_image[cur_row+5][cur_col+5] < 727:
		    if cur_row < min_row:
			min_row = cur_row
		    if cur_row > max_row:
			max_row = cur_row
                    if cur_col < min_col:
			min_col = cur_col
		    if cur_col>max_col:
			max_col = cur_col
	self.depth_image_min_row = min_row
	self.depth_image_max_row = max_row
	self.depth_image_min_col = min_col
	self.depth_image_max_row = max_col
	#print min_row, max_row
	#print min_col, max_col
	board_depth = depth_image[min_row:max_row,min_col:max_col]
	#print np.shape(board_depth)
	board_depth_binary = cv2.inRange(depth_image,0,720) # get binary image of depth frame by the reading of depth camera. 720 is the boundary of bricks surface and surfaces under them.
	im2, contours,hierarchy = cv2.findContours(board_depth_binary,1,2)
	for contour in contours:
	    contour_area = cv2.contourArea(contour)
	    # filter contours by area. Do experiment with Giri on Jan 23rd
	    if contour_area>100 and contour_area<1000:
		rectangle = cv2.minAreaRect(contour)
		#print contour_area
		#print rectangle
		self.blobs_rectangles.append(rectangle)
		box_points = cv2.boxPoints(rectangle)
		box_points = np.int0(box_points)
		self.blobs_box_pts.append(box_points)
		cv2.drawContours(im2,[box_points],0,(255,255,255),2)
	cv2.imwrite('contours.jpg',im2)
	#self.color_detection("green")
	#print "color finished!"

    def color_detection(self, color_str = 'blue'): #Josh

	self.blobs_box_pts_rgb_frame = []
	self.captureVideoFrame()
	RGB_image = self.currentVideoFrame
	hsv_image = cv2.cvtColor(RGB_image, cv2.COLOR_RGB2HSV)
	test_hsv = hsv_image
	for box_points in self.blobs_box_pts:
	    box_pts_rgb = []
	    for point in box_points:
		point = [point[0],point[1],1.]
		rgb_point = np.dot(self.dep2rgb_aff_matrix,np.transpose(point))
		box_pts_rgb.append([rgb_point[0],rgb_point[1]])
	    box_pts_rgb = np.int0(box_pts_rgb)
	    box_pts_rgb = np.array(box_pts_rgb)
	    self.blobs_box_pts_rgb_frame.append(box_pts_rgb)
	    #cv2.drawContours(test_hsv,[box_pts_rgb],0,(255,255,255),2)
	#cv2.imwrite('color_test.jpg',test_hsv)
	if color_str == 'red':
	    upper_bound1 = np.array([180.,255.,255.])
	    lower_bound1 = np.array([170.,0.,80.])
	    upper_bound2 = np.array([5.,255.,255.])
	    lower_bound2 = np.array([0.,0.,80.])
	    count = 0
	    for box_points in self.blobs_box_pts_rgb_frame:
		count += 1
		pt1 = box_points[0]
		pt3 = box_points[2]
		centerx = np.int0((pt1[1] + pt3[1])/2) # box points (output from opencv function) has inverse order of xy compare to our x for row y for column
		centery = np.int0((pt1[0] + pt3[0])/2)
		#centerx = 346
		#centery = 350
		angle_yoverx = np.arctan2((pt1[1] - pt3[1]),(pt1[0] - pt3[0]))
		#cv2.circle(test_hsv,(centerx, centery), 2, (255,255,255), -1)
	    #cv2.imwrite('test_center.jpg',test_hsv)
		window = hsv_image[centerx-3:centerx+4, centery-3:centery+4]
		#print window
		#restore = cv2.cvtColor(window, cv2.COLOR_HSV2BGR)
		#cv2.imwrite('window_restore.jpg',restore)
		mask1 = cv2.inRange(window, lower_bound1, upper_bound1)
		mask2 = cv2.inRange(window, lower_bound2, upper_bound2)
		mask = cv2.bitwise_or(mask1,mask2)
		#print mask
		kernel = np.ones((3,3),np.uint8)
		close = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
		#print centerx, centery
		#print close
		kernel = np.ones((3,3),np.uint8)
		close_sum = sum(map(sum, close))
		if close_sum >= 255*49*0.6:
		    cv2.imwrite('opening {}.jpg'.format(count),close)
		    cv2.imwrite('mask {}.jpg'.format(count),mask)
		    cv2.drawContours(RGB_image,[box_points],0,(255,255,255),2)
		    print color_str + 'found!'
		    cv2.imwrite('color_test.jpg',RGB_image)
		    return [centerx, centery], angle_yoverx
	    
	    
	else:    
	    for color_idx in range(len(self.hsv_colors)):
	        if self.hsv_colors[color_idx] != color_str:
		    continue
		upper_bound = np.array(self.hsv_upper_hsv[color_idx])
		lower_bound = np.array(self.hsv_lower_hsv[color_idx])
		#print lower_bound[0],upper_bound[0]
		count = 0
		for box_points in self.blobs_box_pts_rgb_frame:
		    count += 1
		    pt1 = box_points[0]
		    pt3 = box_points[2]
		    centerx = np.int0((pt1[1] + pt3[1])/2) # box points (output from opencv function) has inverse order of xy compare to our x for row y for column
		    centery = np.int0((pt1[0] + pt3[0])/2)
		    #centerx = 280
		    #centery = 403
		    #self.blobs_center_rgb.append([centerx,centery])
		    angle_yoverx = np.arctan2((pt1[1] - pt3[1]),(pt1[0] - pt3[0]))
		    #cv2.circle(test_hsv,(centerx, centery), 2, (255,255,255), -1)
	    #cv2.imwrite('test_center.jpg',test_hsv)
		    window = hsv_image[centerx-3:centerx+4, centery-3:centery+4]
		    #print window
		    #restore = cv2.cvtColor(window, cv2.COLOR_HSV2BGR)
		    #cv2.imwrite('window_restore.jpg',restore)
		    mask = cv2.inRange(window, lower_bound, upper_bound)
		    #print mask
		    kernel = np.ones((3,3),np.uint8)
		    close = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
		    #print centerx, centery
		    #print close
		    kernel = np.ones((3,3),np.uint8)
		    close_sum = sum(map(sum, close))
		    if close_sum >= 255*49*0.5:
		        #cv2.imwrite('opening {}.jpg'.format(count),close)
		        #cv2.imwrite('mask {}.jpg'.format(count),mask)
			cv2.drawContours(RGB_image,[box_points],0,(255,255,255),2)
			print color_str + 'found!'
			cv2.imwrite('color_test.jpg',RGB_image)
			return [centerx, centery], angle_yoverx
	return [0.,0.], 0.

	












