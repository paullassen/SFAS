# Coordinate systems: (x,y) Robot origin/odometry
					# (w,z) Unknown reference frame 

import math

# This function gives the solutions extracted directly from MATLAB
# inputs: x1, x2 -> x coordinates of the 2 qr codes - robot frame  
#         y1, y2 -> y coordinates of the 2 qr codes - robot frame 
#         (w1,z1) (w2,z2) -> coordinates of the 2 qr codes - unknown frame  

def world_frame_calc(x1, y1, x2, y2, w1, z1, w2, z2):
	l1 = math.sqrt(x1**2 + y1**2) # Distance from unknown frame origin - QR 1
	l2 = math.sqrt(x2**2 + y2**2) # Distance from unknown frame origin - QR 2

	# Two possible solutions for the unknown frame
	# Solution 1
	w01 =  ( - (l1**2 - l2**2 - w1**2 + w2**2 - z1**2 + z2**2)/(2*(w1 - w2)) - 
			 ((z1 - z2)*(w1*((l1**2 + 2*l1*l2 + l2**2 - w1**2 + 2*w1*w2 - w2**2 - z1**2 + 2*z1*z2 - z2**2)*
			 (- l1**2 + 2*l1*l2 - l2**2 + w1**2 - 2*w1*w2 + w2**2 + z1**2 - 2*z1*z2 + z2**2))**(1/2) 
			 - w2*((l1**2 + 2*l1*l2 + l2**2 - w1**2 + 2*w1*w2 - w2**2 - z1**2 + 2*z1*z2 - z2**2)*
			 (- l1**2 + 2*l1*l2 - l2**2 + w1**2 - 2*w1*w2 + w2**2 + z1**2 - 2*z1*z2 + z2**2))**(1/2) 
			 - l1**2*z1 + l1**2*z2 + l2**2*z1 - l2**2*z2 + w1**2*z1 + w1**2*z2 + w2**2*z1 + w2**2*z2 - z1*z2**2 - z1**2*z2 
			 + z1**3 + z2**3 - 2*w1*w2*z1 - 2*w1*w2*z2))/(2*(w1 - w2)*(w1**2 - 2*w1*w2 + w2**2 + z1**2 - 2*z1*z2 + z2**2)) )

	z01 = ( (w1*((l1**2 + 2*l1*l2 + l2**2 - w1**2 + 2*w1*w2 - w2**2 - z1**2 + 2*z1*z2 - z2**2)*
			(- l1**2 + 2*l1*l2 - l2**2 + w1**2 - 2*w1*w2 + w2**2 + z1**2 - 2*z1*z2 + z2**2))**(1/2) 
			- w2*((l1**2 + 2*l1*l2 + l2**2 - w1**2 + 2*w1*w2 - w2**2 - z1**2 + 2*z1*z2 - z2**2)*
			(- l1**2 + 2*l1*l2 - l2**2 + w1**2 - 2*w1*w2 + w2**2 + z1**2 - 2*z1*z2 + z2**2))**(1/2) 
			- l1**2*z1 + l1**2*z2 + l2**2*z1 - l2**2*z2 + w1**2*z1 + w1**2*z2 + w2**2*z1 + w2**2*z2 
			- z1*z2**2 - z1**2*z2 + z1**3 + z2**3 - 2*w1*w2*z1 - 2*w1*w2*z2)/(2*(w1**2 - 2*w1*w2 
			+ w2**2 + z1**2 - 2*z1*z2 + z2**2)) )

	# Solution 2
	w02 =   ( - (l1**2 - l2**2 - w1**2 + w2**2 - z1**2 + z2**2)/(2*(w1 - w2)) - 
			((z1 - z2)*(w2*((l1**2 + 2*l1*l2 + l2**2 - w1**2 + 2*w1*w2 - w2**2 - z1**2 + 2*z1*z2 - z2**2)*
			(- l1**2 + 2*l1*l2 - l2**2 + w1**2 - 2*w1*w2 + w2**2 + z1**2 - 2*z1*z2 + z2**2))**(1/2) 
			- w1*((l1**2 + 2*l1*l2 + l2**2 - w1**2 + 2*w1*w2 - w2**2 - z1**2 + 2*z1*z2 - z2**2)*
			(- l1**2 + 2*l1*l2 - l2**2 + w1**2 - 2*w1*w2 + w2**2 + z1**2 - 2*z1*z2 + z2**2))**(1/2) 
			- l1**2*z1 + l1**2*z2 + l2**2*z1 - l2**2*z2 + w1**2*z1 + w1**2*z2 + w2**2*z1 + w2**2*z2 - z1*z2**2 - z1**2*z2 
			+ z1**3 + z2**3 - 2*w1*w2*z1 - 2*w1*w2*z2))/(2*(w1 - w2)*(w1**2 - 2*w1*w2 + w2**2 + z1**2 - 2*z1*z2 + z2**2)) )


	z02 = ( (w2*((l1**2 + 2*l1*l2 + l2**2 - w1**2 + 2*w1*w2 - w2**2 - z1**2 + 2*z1*z2 - z2**2)*
			(- l1**2 + 2*l1*l2 - l2**2 + w1**2 - 2*w1*w2 + w2**2 + z1**2 - 2*z1*z2 + z2**2))**(1/2) 
			- w1*((l1**2 + 2*l1*l2 + l2**2 - w1**2 + 2*w1*w2 - w2**2 - z1**2 + 2*z1*z2 - z2**2)*
			(- l1**2 + 2*l1*l2 - l2**2 + w1**2 - 2*w1*w2 + w2**2 + z1**2 - 2*z1*z2 + z2**2))**(1/2) 
			- l1**2*z1 + l1**2*z2 + l2**2*z1 - l2**2*z2 + w1**2*z1 + w1**2*z2 + w2**2*z1 + w2**2*z2 
			- z1*z2**2 - z1**2*z2 + z1**3 + z2**3 - 2*w1*w2*z1 - 2*w1*w2*z2)/(2*(w1**2 - 2*w1*w2 + w2**2 
			+ z1**2 - 2*z1*z2 + z2**2)) )

	world_frame = [w01, z01, w02, z02]
	return world_frame
#####################################################################################################################

# This function calculates the orientation of the world frame
# inputs: (x,y) -> Coordinates according to the world frame
#         (w,z) -> Coordinates according to the robot frame
#         (w0,z0) -> Coordinates of world's frame origin according to the robot frame

def rot(x,y,w,z,w0,z0):
	theta = math.atan((z-z0)/(w-w0)) - math.atan(y/x)	
	return theta
#####################################################################################################################

# Test 1 #
x1 = -10
y1 = -2
x2 = -3
y2 = -10
w1 = 5
z1 = 6
w2 = 12
z2 = -2

world_frame_candidates = world_frame_calc(x1, y1, x2, y2, w1, z1, w2, z2)
print (world_frame_candidates)

#QR1 - first candidate
rot1 = rot(x1,y1,w1,z1,world_frame_candidates[0],world_frame_candidates[1])
print(math.degrees(rot1))

#QR2 - first candidate
rot1 = rot(x2,y2,w2,z2,world_frame_candidates[0],world_frame_candidates[1])
print(math.degrees(rot1))

#QR1 - second candidate
rot2 = rot(x1,y1,w1,z1,world_frame_candidates[2],world_frame_candidates[3])
print(math.degrees(rot2))

#QR2 - second candidate
rot2 = rot(x2,y2,w2,z2,world_frame_candidates[2],world_frame_candidates[3])
print(math.degrees(rot2))
###########################################################################

# Test 2 #
x1 = -33
y1 = 8
x2 = 3
y2 = 24
w1 = -5
z1 = -6
w2 = 31
z2 = 10

world_frame_candidates = world_frame_calc(x1, y1, x2, y2, w1, z1, w2, z2)
print (world_frame_candidates)

#QR1 - first candidate
rot1 = rot(x1,y1,w1,z1,world_frame_candidates[0],world_frame_candidates[1])
print(math.degrees(rot1))

#QR2 - first candidate
rot1 = rot(x2,y2,w2,z2,world_frame_candidates[0],world_frame_candidates[1])
print(math.degrees(rot1))

#QR1 - second candidate
rot2 = rot(x1,y1,w1,z1,world_frame_candidates[2],world_frame_candidates[3])
print(math.degrees(rot2))

#QR2 - second candidate
rot2 = rot(x2,y2,w2,z2,world_frame_candidates[2],world_frame_candidates[3])
print(math.degrees(rot2))
###########################################################################

# Test 3 #
# Let's add -45 degrees rotation anti-clockwise #
x1 = 12.7279
y1 = 7.0711
x2 = -12.0208
y2 = 10.6066
w1 = -4
z1 = 18
w2 = -19
z2 = -2

world_frame_candidates = world_frame_calc(x1, y1, x2, y2, w1, z1, w2, z2)
print (world_frame_candidates)

#QR1 - first candidate
rot1 = rot(x1,y1,w1,z1,world_frame_candidates[0],world_frame_candidates[1])
print(math.degrees(rot1))

#QR2 - first candidate
rot1 = rot(x2,y2,w2,z2,world_frame_candidates[0],world_frame_candidates[1])
print(math.degrees(rot1))

#QR1 - second candidate
rot2 = rot(x1,y1,w1,z1,world_frame_candidates[2],world_frame_candidates[3])
print(math.degrees(rot2))

#QR2 - second candidate
rot2 = rot(x2,y2,w2,z2,world_frame_candidates[2],world_frame_candidates[3])
print(math.degrees(rot2))
###########################################################################

# Test 4 #
# +45 degrees rotation #
y1 = 12.7279
x1 = 7.0711
y2 = -12.0208
x2 = 10.6066
w1 = -4
z1 = 18
w2 = -19
z2 = -2

world_frame_candidates = world_frame_calc(x1, y1, x2, y2, w1, z1, w2, z2)
print (world_frame_candidates)

#QR1 - first candidate
rot1 = rot(x1,y1,w1,z1,world_frame_candidates[0],world_frame_candidates[1])
print(math.degrees(rot1))

#QR2 - first candidate
rot1 = rot(x2,y2,w2,z2,world_frame_candidates[0],world_frame_candidates[1])
print(math.degrees(rot1))

#QR1 - second candidate
rot2 = rot(x1,y1,w1,z1,world_frame_candidates[2],world_frame_candidates[3])
print(math.degrees(rot2))

#QR2 - second candidate
rot2 = rot(x2,y2,w2,z2,world_frame_candidates[2],world_frame_candidates[3])
print(math.degrees(rot2))
