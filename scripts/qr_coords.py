#!/usr/bin/env python
import rospy
import math
#from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg as geo
import std_msgs.msg as stm
import tf2_msgs.msg as tfm
import tf.transformations as tft

from genious_math import unknown_frame

class qr_store:
	def __init__(self,known_loc, unknown_loc, unknown_next, num, letter):
		self.world_loc = known_loc
		self.frame_loc = unknown_loc
		self.next_qr = unknown_next
		self.num = num
		self.letter = letter
		self.robot = []

class Wasp:
	def __init__(self):
		self.coords= [0.0,0.0]
		self.status=0	
		self.odom = [0.0,0.0]
		self.rotation = 0
		self.qr_coords = [0.0, 0.0]
		self.msg = []
		self.qr_store = [0, 0, 0, 0, 0]
		self.flag  = 0
		self.nums = []
		self.frame_origin = []
		self.max_cov = 1
		self.flaggs = 0
		self.done = False
	def update_pos(self,msg):
		pos = msg.pose.pose.position
		cov = msg.pose.covariance
		self.max_cov = max(cov)
		self.coords[0] = math.sqrt(0.0**2 + pos.z**2)
		self.coords[1] = -pos.x + 0.0025
		self.qr_coords[0] = self.odom[0] + self.coords[0]*math.cos(self.rotation) - self.coords[1]*math.sin(self.rotation)
		self.qr_coords[1] = self.odom[1] + self.coords[0]*math.sin(self.rotation) + self.coords[1]*math.cos(self.rotation)
		self.flag = 1
	
	def update_status(self,msg):
		self.status = msg.data
		self.print_coords()
		#print 'x: {}\ty: {}\t | status: {}'.format(self.coords[0],self.coords[1], self.status)
		#print '\033[2A\r'

	def update_odom(self, msg):		
		pos = msg.pose.pose.position
		self.odom[0] = pos.x
		self.odom[1] = pos.y
		rot = msg.pose.pose.orientation
		eul = tft.euler_from_quaternion((rot.x,rot.y, rot.z, rot.w)) 
		self.rotation = eul[2]
			#print 'x: {}\ty: {}\t ================'.format(self.odom[0],self.odom[1])

	def update(self, msg, var):
		if var == 1:
			self.update_pos(msg)
		elif var == 2:
			self.update_status(msg)
		elif  var == 3:
			self.update_odom(msg)
		elif var == 4:
			self.add_qr(msg)
		elif var == 5:
			self.flagging(msg)

	def add_qr(self, msg):
		if msg.data not in self.msg and self.flag > 0 and msg.data != '' and self.max_cov < 3e-04 and self.flaggs == 1: 
			self.msg.append(msg.data)
			self.handle_message(msg.data, (self.qr_coords[0], self.qr_coords[1]))
			self.flag = 0
			if len(self.nums) == 2:
				tmp = []
				for num in self.nums:
					tmp.append(self.qr_store[num].world_loc)
					tmp.append(self.qr_store[num].frame_loc)
				self.frame_origin = unknown_frame(tmp[1][0], tmp[1][1], tmp[3][0], tmp[3][1], tmp[0][0], tmp[0][1], tmp[2][0], tmp[2][1])
				#self.frame_origin[0][1] = -self.frame_origin[0][1]
	
	def print_coords(self):
                msg = ''
		for num in self.nums:
			msg = msg+ self.qr_store[num].letter
			print self.qr_store[num].world_loc
			print self.qr_store[num].frame_loc
			print "==========="
		if len(self.nums) >= 2:
			print self.frame_origin
			print "----------"
		print msg

	def print_message(self):
		msg = 'Message: '
		for qrs in self.qr_store:
			msg = msg + qrs.letter
		print msg

	def flagging(self, msg):
		self.flaggs= msg.data		
	def handle_message(self, data, known_loc):
		inf=data.split("\r\n")
		unknown_loc = (float(inf[0][2:]),  float(inf[1][2:]))
		unknown_next = (float(inf[2][7:]), float(inf[3][7:]))
		num = int(inf[4][2:])-1
		letter = inf[5][2:]
		self.nums.append(num)
		self.qr_store[num] = qr_store(known_loc, unknown_loc, unknown_next, num, letter)
		self.qr_store[num].robot = (self.coords[0], self.coords[1])
	def next_loc(self):
		ind = min(self.nums)

		while ind in self.nums:
			ind = (ind + 1) % 5

		nxt = self.qr_store[ind-1].next_qr
		nxt_x = self.frame_origin[0][0]+nxt[0]*math.cos(self.frame_origin[0][2])-math.sin(self.frame_origin[0][2])*nxt[1]
		nxt_y = self.frame_origin[0][1]+nxt[0]*math.sin(self.frame_origin[0][2])+math.cos(self.frame_origin[0][2])*nxt[1]
		nxt = (nxt_x, nxt_y, ind+1)
		return nxt	

	def pub_loc(self,publisher):
		pub = geo.Pose()
		if len(self.nums)<2:
			pub.position.x = -10
			pub.position.y = -10
			pub.position.z = -10
		elif len(self.nums)>=5:
			pub.position.x = 10
			pub.position.y = 10
			pub.position.z = 10
			self.done = True
		else:
			nxt = self.next_loc()
			pub.position.x = nxt[0]
			pub.position.y = nxt[1]

			if pub.position.x > 3.5:
				pub.position.x = 3.5
			elif pub.position.x < 0.0:
				pub.position.x = 0.0
			
			if pub.position.y > 3.3:
				pub.position.y = 3.3
			elif pub.position.y < -2.0:
				pub.position.y = -2.0

			eul = (0.0,0.0,math.atan2(nxt[1]-1, nxt[0]-2))
			quat = tft.quaternion_from_euler(0.0,0.0, math.atan2(nxt[1],nxt[0]))
			pub.orientation.x = quat[0]
			pub.orientation.y = quat[1]
			pub.orientation.z = quat[2]
			pub.orientation.w = quat[3]
		publisher.publish(pub)
def visp_callback(msg, callback_arg):
	some_code = msg
	wsp = callback_arg[0]
	var = callback_arg[1]
	wsp.update(msg, var)
if __name__ == '__main__':
	pub = rospy.Publisher('nxt_loc', geo.Pose, queue_size=1)
	rospy.init_node('wasp')
	wsp = Wasp()	
	
	pos_var = 1
	status_var = 2
	tf_var = 3
	message_var = 4
	twist_var = 5	

	rate = rospy.Rate(3)	
	rospy.Subscriber("visp_auto_tracker/object_position_covariance",geo.PoseWithCovarianceStamped,visp_callback, (wsp, pos_var))
	rospy.Subscriber("visp_auto_tracker/status",stm.Int8,visp_callback, (wsp, status_var))
	rospy.Subscriber("visp_auto_tracker/code_message",stm.String,visp_callback, (wsp, message_var))
	rospy.Subscriber("amcl_pose",geo.PoseWithCovarianceStamped, visp_callback, (wsp, tf_var))
	rospy.Subscriber("/flags_for_days", stm.Int8, visp_callback, (wsp, twist_var))
	while not rospy.is_shutdown() and not wsp.done:
		wsp.pub_loc(pub)
		rate.sleep()
	wsp.print_message()
