#!/usr/bin/env python
import rospy

#from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg as geo
import std_msgs.msg as stm
import tf2_msgs.msg as tfm

class Wasp:
	def __init__(self):
		self.coords= [0.0,0.0]
		self.status=0	
		self.odom = [0.0, 0.0]
	def update_pos(self,msg):
		pos = msg.pose.position
		self.coords[0] = pos.z
		self.coords[1] = -pos.x
	
	def update_status(self,msg):
		self.status = msg.data
		#print 'x: {}\ty: {}\t | status: {}'.format(self.coords[0],self.coords[1], self.status)
		#print '\033[2A\r'

	def update_odom(self, msg):		
		for transform in msg.transforms:
			if transform.child_frame_id == "base_footprint":
				self.odom[0] = transform.transform.translation.x
				self.odom[1] = transform.transform.translation.y
				#print 'x: {}\ty: {}\t ================'.format(self.odom[0],self.odom[1])

	def update(self, msg, var):
		if var == 1:
			self.update_pos(msg)
		elif var == 2:
			self.update_status(msg)
		elif  var == 3:
			self.update_odom(msg)

	def print_coords(self):
		print '========================================'
		print 'Current Location:' 
		print '========================================'
		print '\tx: {}\n\ty: {}'.format(self.odom[0], self.odom[1])
		print '========================================'
		if self.status > 2:
			print 'QR Location:      Status: Visible'
		else:
			print 'QR Location:      Status: invisible'	
		print '========================================'
		print '\tx: {}\n\ty: {}'.format(self.odom[0]+self.coords[0], self.odom[1]+self.coords[1])
		print '\033[11A\r'

def visp_callback(msg, callback_arg):
	some_code = msg
	wsp = callback_arg[0]
	var = callback_arg[1]
	wsp.update(msg, var)
	wsp.print_coords()
if __name__ == '__main__':
	rospy.init_node('wasp')
	
	wsp = Wasp()	
	
	pos_var = 1
	status_var = 2
	tf_var = 3

	rospy.Subscriber("visp_auto_tracker/object_position",geo.PoseStamped,visp_callback, (wsp, pos_var))
	rospy.Subscriber("visp_auto_tracker/status",stm.Int8,visp_callback, (wsp, status_var))
	rospy.Subscriber("tf",tfm.TFMessage, visp_callback, (wsp, tf_var))
	rospy.spin()
