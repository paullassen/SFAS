# Coordinate systems: (x,y) Robot origin/odometry
					# (w,z) Unknown reference frame 

import math

# This function gives the solutions extracted directly from MATLAB
# inputs: (x1,y1) (x2,y2) -> QR coords - unknown frame
#         (w1,z1) (w2,z2) -> QR coords - world frame

def unknown_frame_calc(x1, y1, x2, y2, w1, z1, w2, z2):
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

	unknown_cand = [w01, z01, w02, z02]
	return unknown_cand
#####################################################################################################################

# This function calculates the orientation of the unknown frame
# inputs: (x1,y1) (x2,y2) -> QR coords - unknown frame
#         (w1,z1) (w2,z2) -> QR coords - world frame
#         (w0,z0) -> Coordinates of the unknown's frame origin according to the world frame

def rot(x1, y1, x2, y2, w1, z1, w2, z2, w0, z0):
    # limit -pi pi
    theta1 = math.asin(((z1-z0)*x1 - y1*(w1-w0))/((w1-w0)**2+(z1-z0)**2))
    sign = 1 if (theta1>=0) else -1	
    if abs(theta1) > math.pi:
        theta1 = theta1 - 2*math.pi*sign

    theta2 = math.asin(((z2-z0)*x2 - y2*(w2-w0))/((w2-w0)**2+(z2-z0)**2))
    sign = 1 if (theta2>=0) else -1	
    if abs(theta2) > math.pi:
        theta2 = theta2 - 2*math.pi*sign

    if theta1 == 0 and theta2 == 0:
        x1pi = (w1-w0)*math.cos(math.pi)+(z1-z0)*math.sin(math.pi)
        y1pi = -(w1-w0)*math.sin(math.pi)+(z1-z0)*math.cos(math.pi)
        if round(x1pi,4) == round(x1,4) and round(y1pi,4) == round(y1,4):
            theta1 = math.pi
            theta2 = math.pi
        else:
            theta1 = 0
            theta2 = 0

    rot = [theta1, theta2]

    return rot
#####################################################################################################################

def unknown_frame(x1, y1, x2, y2, w1, z1, w2, z2):
	unknown_cand = unknown_frame_calc(x1, y1, x2, y2, w1, z1, w2, z2)
	print(unknown_cand)
	rot1 = rot(x1, y1, x2, y2, w1, z1, w2, z2, unknown_cand[0], unknown_cand[1])
	print(rot1)
	rot2 = rot(x1, y1, x2, y2, w1, z1, w2, z2, unknown_cand[2], unknown_cand[3])
	print(rot2)

	if round(rot1[0],4) == round(rot1[1],4):
		unknown_frame = {"w0" : unknown_cand[0], "z0" : unknown_cand[1], "theta" : rot1[0]}
		print(unknown_frame)
	elif round(rot2[0],4) == round(rot2[1],4):
		unknown_frame = {"w0" : unknown_cand[2], "z0" : unknown_cand[3], "theta" : rot2[0]}
		print(unknown_frame)
	else:
		print ("Whoops! You fucked up somewhere!")
#####################################################################################################################