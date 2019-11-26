#!/usr/bin/env python
 
import rospy
import actionlib
from tf.transformations import quaternion_from_euler
import std_msgs.msg as stm
import actionlib_msgs.msg as alm
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
 
explore = True

waypoints = [  
    [(2.3, 2.3, 0.0), (0, 0, 0.7071068, 0.7071068)], #1
    [(2.3, 2.3, 0.0), (0, 0, 1, 0)],
    [(4.3, 2.3, 0.0), (0, 0, 0, 1)], #2
    [(4.3, 2.3, 0.0), (0, 0, 0.7071068, 0.7071068)],
    [(4.3, -1.3, 0.0), (0, 0, -0.7071068, 0.7071068)], #3
    [(4.3, -1.3, 0.0), (0, 0, 0, 1)],
    [(2.3, -1.3, 0.0), (0, 0, 1, 0)],#4
    [(2.3, -1.3, 0.0), (0, 0, -0.7071068, 0.7071068)], 
    [(3.3, 2.3, 0.0), (0, 0, 0.7071068, 0.7071068)], #5
    [(4.3, 0, 0.0), (0, 0, 0, 1)], #6
    [(3.3, -1,3, 0.0), (0, 0, -0.7071068, 0.7071068)], #7
    [(3.3, 0, 0.0), (0, 0, 0.7071068, 0.7071068)], #8 middle of room
    [(3.3, 0.2, 0.0), (0, 0, -0.7071068, 0.7071068)],
    [(3.3, -0.2, 0.0), (0, 0, 0.7071068, 0.7071068)]


#    [(0.13, 1.93, 0.0), (0.0, 0.0, -0.64003024, -0.76812292098)]
]
 
 
def goal_pose(pose):  
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
 
    return goal_pose

class Wasp:
    def __init__(self):
        self.qr_status = 0
        self.goal_status = 0
    def set_qr_status(self, status):
        self.qr_status = status

    def set_goal_status(self, status):
        self.goal_status = status

    def get_qr_status(self):
        return self.qr_status

    def get_goal_status(self):
        return self.goal_status

def visp_callback(msg,callback_arg):
    callback_arg.set_qr_status(msg.data)

def goal_callback(msg,callback_arg):
    status_list = msg.status_list
    length = len(status_list)
    if length > 0:
        callback_arg.set_goal_status(status_list[length-1].status)
if __name__ == '__main__':
    rospy.init_node('patrol')

    wsp = Wasp()

    rospy.Subscriber("visp_auto_tracker/status",stm.Int8,visp_callback, wsp)
    rospy.Subscriber("move_base/status",alm.GoalStatusArray,goal_callback, wsp)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()

    end = 0

    for pose in waypoints:   
        client.stop_tracking_goal()
        goal = goal_pose(pose)
        client.send_goal(goal)
        goal_state = wsp.get_goal_status()
        while goal_state != 3:
            goal_state = wsp.get_goal_status()
            qr_status = wsp.get_goal_status()

            #if qr_status > 2:
             #   print "QR code"
              #  end = 1
               # break

        rospy.sleep(3)

#        if end:
 #           break

    print "Done"
    rospy.spin()

