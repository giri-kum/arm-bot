from numpy import *

#constants(cm):
#---------------------------------------------------------------------------
d1=12.1#11.7 #link1 length
alpha1=-90 #link1 twist
a2=10 #link2 length
a3=10.1 #link3 length
a4=13#10.86#11.1#11.1 before#11 #link4 length
origin_offset = [0.7,-1,0] #in cm
debug = False
#Inverse kinematics accepts variables in cm and degrees!
def inverseKinematics(x,y,z,phi):
	if(debug):	
		print "x,y,z from IK:" + str(x) + "," + str(y) + "," + str(z)
	x = x+origin_offset[0]
	y = y+origin_offset[1] 		
	z=z-d1
    	theta1=90+arctan2(y,x)*180/pi #base angle
	r=sqrt(x**2+y**2) #hypotenuse of x and y

	#rw=r-a4*abs(sin(phi*pi/180)) #r coordinate of point w
	if x>=0: 
		rw=r-a4*sin(phi*pi/180)
		zw=z-a4*cos(phi*pi/180) #z coordinate of point w
	else:
		rw=r-a4*sin(phi*pi/180)
		zw=z-a4*cos(phi*pi/180)
		#phi=-phi
		#zw=z-a4*abs(cos(phi*pi/180)) #z coordinate of point w
	if(debug):
		print "r: " + str(r)
		print "a: " + str(a4*sin(phi*pi/180))		
		print "rw: " + str(rw)
				
	alpha=arctan2(zw,rw) #used to calculate other angles is in radians
	betaarg=(rw**2 +zw**2 +a2**2-a3**2)/(2*a2*sqrt(rw**2 +zw**2) )
	if betaarg>=-1 and betaarg<=1:
		pass
	else:
		print "Pose cannot be reached. Betaarg is outside of -1 to 1, betaarg = ", betaarg
		return array([0,0,0,0])
	#if betaarg is less than 0, or greater than pi, handle problem.
	beta=arccos(betaarg) #beta is in radians
	if x>=0:
	      	theta2=90-(alpha+beta)*180/pi #shoulder angle in degrees
	else:
		theta2=90-(alpha+beta)*180/pi #dubious, please check.

	theta3arg=(a2**2 +a3**2 -rw**2 -zw**2)/(2*a2*a3)
	if theta3arg>=-1 and theta3arg<=1:
		pass
	else:
		print "Pose cannot be reached. Theta3arg is outside of -1 to 1, theta3arg = ", theta3arg
		return array([0,0,0,0])

	#if theta3arg is less than 0, or greater than pi, handle problem.
	if x>=0:
		theta3=(pi-arccos(theta3arg))*180/pi #elbow angle in degrees
	else:
		theta3=(pi-arccos(theta3arg))*180/pi
	
	theta4=phi-theta2-theta3 #wrist angle.

	if abs(theta2)>128:
	    print "theta2 is greater than 128 degrees. Limit on shoulder is 128 degrees, theta2 = ", theta2
	    return array([0,0,0,0])

	if abs(theta3)>121:
	    print "theta3 is greater than 121 degrees. Limit on elbow is 121 degrees, theta3 = ", theta3
	    return array([0,0,0,0])
	if abs(theta4)>120:
	    print "theta4 is greater than 121 degrees. Limit on wrist is 121 degrees, theta4 = ", theta4
	    return array([0,0,0,0])
	
	if(debug):
		print "Output of IK = ", [theta1,theta2,theta3,theta4]
	return array([theta1,theta2,theta3,theta4])
'''
forwardKinematics:
Calculates a vector of end effector values using Davenit Hartenberg convention. 
jointAngle1, jointAngle2, jointAngle3, jointAngle4 are in degrees. End Effector
is a 1x4 vector with the following values:
	1.) x
	2.) y
	3.) z
	4.) phi (the angle of the end effector from the horizontal plane)
x,y,z are in cm!
'''
#DH- Davenit Hartenberg naming convention.
#DH_zero position in real coordinates: [90,90,0,0]
#real frame zero position in DH: [-90,-90,0,0]
def forwardKinematics(jointAngle1, jointAngle2, jointAngle3, jointAngle4):
	dh_Angles=worldFrame_to_DH(jointAngle1,jointAngle2,jointAngle3,jointAngle4)
	dh_angle1=dh_Angles[0]
	dh_angle2=dh_Angles[1]
	dh_angle3=dh_Angles[2]
	dh_angle4=dh_Angles[3]

	#phi=dh_angle1+dh_angle2+dh_angle3+dh_angle4
	phi=jointAngle2+jointAngle3+jointAngle4
	A1=transformation1(dh_angle1)
	A2=transformation2(dh_angle2)
	A3=transformation3(dh_angle3)
	A4=transformation4(dh_angle4)
	T=A1.dot(A2).dot(A3).dot(A4)
	x=T[0,3]
	y=T[1,3]
	z=T[2,3]
	return array([x,y,z,phi])

def worldFrame_to_DH(jointAngle1, jointAngle2, jointAngle3, jointAngle4):
	#dh_Angles=[jointAngle1-90,jointAngle2-90,jointAngle3,jointAngle4]
	dh_Angles=[jointAngle1-90,jointAngle2-90,jointAngle3,jointAngle4]
	return dh_Angles


def transformation1(jointAngle1):
	A1=zRotationMat(jointAngle1).dot(zTranslationMat(d1)).dot(xRotationMat(alpha1))
	return A1

def transformation2(jointAngle2):
	#note that there is a twist on link 2 relative to "link" 1, a 
	alpha2=0
	A2=zRotationMat(jointAngle2).dot(xTranslationMat(a2)).dot(xRotationMat(alpha2))
	return A2

def transformation3(jointAngle3):
	A3=zRotationMat(jointAngle3).dot(xTranslationMat(a3))
	return A3

def transformation4(jointAngle4):
	A4=zRotationMat(jointAngle4).dot(xTranslationMat(a4))
	return A4

def zRotationMat(jointAngle):
	R=identity(4)
	R[0,0]=cos(jointAngle*pi/180)
	R[0,1]=-1*sin(jointAngle*pi/180)
	R[1,0]=sin(jointAngle*pi/180)
	R[1,1]=cos(jointAngle*pi/180)
	return R

def zTranslationMat(link_length):
	T=identity(4)
	T[2,3]=link_length
	return T

def xRotationMat(jointAngle):
	R=identity(4)
	R[1,1]=cos(jointAngle*pi/180)
	R[1,2]=-1*sin(jointAngle*pi/180)
	R[2,1]=sin(jointAngle*pi/180)
	R[2,2]=cos(jointAngle*pi/180)
	return R

def xTranslationMat(link_length):
	T=identity(4)
	T[0,3]=link_length
	return T



def add360ToNegativeAngle(angle):
	if angle<0:
		angle=angle+360
	return angle

def chooseWristRotation(sideAngle, base):
	#sideAngle=add360ToNegativeAngle(sideAngle)
	#base=add360ToNegativeAngle(base)
	wristAngle=min(abs(sideAngle-base),abs(sideAngle-(180+base)) )
	if wristAngle== abs(sideAngle-base):
		return sideAngle-base
	else:
		return sideAngle-(180+base)

def isCloseToVertical(angle):
	if angle<0:
		angle=angle+360
	if angle<45:
		return True
	if angle>135 and angle<225:
		return True
	if angle>330 and angle<360:
		return True
	return False

def isCloseToHorizontal(angle):
	return not isCloseToVertical(angle)

def orientation(base,diagonal):
	phi_arm=diagonal+90
	print "Phi_arm = " ,phi_arm
	if phi_arm>180:
		phi_arm=phi_arm-90
	#side1angle=(phi_arm-45)-base
	#side2angle=abs((phi_arm+45)-base)
	#wristAngle=min(abs((phi_arm-45)-base),abs(base-phi_arm+45))
	#if wristAngle ==abs((phi_arm-45)-base):
	#	return (phi_arm-45)-base
	#else:
	#	return(phi_arm+45)-base

	if isCloseToVertical(base):
		if isCloseToVertical(phi_arm-45):
			return chooseWristRotation(phi_arm-45,base)
		else:
			return chooseWristRotation(phi_arm+45,base)
	if isCloseToHorizontal(base):
		if isCloseToHorizontal(phi_arm-45):
			return chooseWristRotation(phi_arm-45,base)
		else:
			return chooseWristRotation(phi_arm+45,base)

