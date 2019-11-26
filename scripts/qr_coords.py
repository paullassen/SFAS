#!/usr/bin/env python
import rospy
import math
#from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg as geo
import std_msgs.msg as stm
import tf2_msgs.msg as tfm
import tf.transformations as tft
class Wasp:
	def __init__(self):
		self.coords= [0.0,0.0]
		self.status=0	
		self.odom = [0.0, 0.0]
		self.rotation = 0
		self.qr_coords = [0.0, 0.0]
		self.qrs = {}
		self.flag  = 0
	def update_pos(self,msg):
		pos = msg.pose.position
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

	def add_qr(self, msg):
		if msg.data not in self.qrs and self.flag > 0 and msg.data != '':
			self.qrs[msg.data] = (self.qr_coords[0], self.qr_coords[1])
			self.flag = 0

	def print_coords(self):
		print self.qrs
"""
		print '\r========================================'
		print '\rCurrent Location:' 
		print '\r========================================'
		print '\r\tx: {}\n\ty: {}'.format(self.odom[0], self.odom[1])
		print '\r========================================'
		if self.status > 2:
			print '\rQR Location:      Status: Visible'
		else:
			print '\rQR Location:      Status: Invisible'	
		print '\r========================================'
		print '\r\tx: {}\n\ty: {}'.format(self.qr_coords[0], self.qr_coords[1])
		#print '\r\033[10A'
"""
def visp_callback(msg, callback_arg):
	some_code = msg
	wsp = callback_arg[0]
	var = callback_arg[1]
	wsp.update(msg, var)
if __name__ == '__main__':
	rospy.init_node('wasp')
	
	wsp = Wasp()	
	
	pos_var = 1
	status_var = 2
	tf_var = 3
	message_var = 4
	rospy.Subscriber("visp_auto_tracker/object_position",geo.PoseStamped,visp_callback, (wsp, pos_var))
	rospy.Subscriber("visp_auto_tracker/status",stm.Int8,visp_callback, (wsp, status_var))
	rospy.Subscriber("visp_auto_tracker/code_message",stm.String,visp_callback, (wsp, message_var))
	rospy.Subscriber("amcl_pose",geo.PoseWithCovarianceStamped, visp_callback, (wsp, tf_var))
	rospy.spin()
