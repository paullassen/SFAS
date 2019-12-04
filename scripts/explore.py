#!/usr/bin/env python
 
import rospy
import actionlib
from tf.transformations import quaternion_from_euler
import std_msgs.msg as stm
import actionlib_msgs.msg as alm
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import geometry_msgs.msg as geo
import tf.transformations as tft
import sensor_msgs.msg as laser
import math

 
stop_explore = False

waypoints = [ 
    [(1, 3.0, 0.0), (0, 0, 0.7071068, 0.7071068)],  
    [(1, 3.0, 0.0), (0, 0, 1, 0)],#7

    [(2.0, 3.0, 0.0), (0, 0, 0.7071068, 0.7071068)], #6

    [(3.5, 3.0, 0.0), (0, 0, 0, 1)],
    [(3.5, 3.0, 0.0), (0, 0, 0.7071068, 0.7071068)], #5

    [(3.5, 0.0, 0.0), (0, 0, 0, 1)], #4
    
    [(3.5, -1.5, 0.0), (0, 0, -0.7071068, 0.7071068)],
    [(3.5, -1.5, 0.0), (0, 0, 0, 1)], #3

    [(2, -1.5, 0.0), (0, 0, -0.7071068, 0.7071068)], #2
    
    [(1, -1.5, 0.0), (0, 0, 1, 0)],
    [(1, -1.5, 0.0), (0, 0, -0.7071068, 0.7071068)], #1

]

# change the waypoints for new map
"""
waypoints = [ 
    
    [(1.0, 3.0, 0.0), (0, 0, 0.7071068, 0.7071068)],  
    [(1.0, 3.0, 0.0), (0, 0, 0.7071068, 0.7071068)],  
    [(1.0, -2.0, 0.0), (0, 0, -0.7071068, 0.7071068)],
    [(1.0, -2.0, 0.0), (0, 0, -0.7071068, 0.7071068)],
    [(2.0, 3.0, 0.0), (0, 0, 0.7071068, 0.7071068)],  
    [(2.0, 3.0, 0.0), (0, 0, 0.7071068, 0.7071068)],  
    [(2.0, -2.0, 0.0), (0, 0, -0.7071068, 0.7071068)],
    [(2.0, -2.0, 0.0), (0, 0, -0.7071068, 0.7071068)],
    [(3.0, 3.0, 0.0), (0, 0, 0.7071068, 0.7071068)],  
    [(3.0, 3.0, 0.0), (0, 0, 0.7071068, 0.7071068)],  
    [(3.0, -2.0, 0.0), (0, 0, -0.7071068, 0.7071068)],
    [(3.0, -2.0, 0.0), (0, 0, -0.7071068, 0.7071068)],  
    [(3.5, 3.0, 0.0), (0, 0, 0.7071068, 0.7071068)],  
    [(3.5, 3.0, 0.0), (0, 0, 0.7071068, 0.7071068)],  
    [(3.5, -2.0, 0.0), (0, 0, -0.7071068, 0.7071068)],
    [(3.5, -2.0, 0.0), (0, 0, -0.7071068, 0.7071068)],  
    
    [(1, 3.0, 0.0), (0, 0, 1, 0)],
    [(1, 3.0, 0.0), (0, 0, 1, 0)],
    [(3.5, 3.0, 0.0), (0, 0, 0, 1)],
    [(3.5, 3.0, 0.0), (0, 0, 0, 1)],
    [(1, 2.0, 0.0), (0, 0, 1, 0)],
    [(1, 2.0, 0.0), (0, 0, 1, 0)],
    [(3.5, 2.0, 0.0), (0, 0, 0, 1)],
    [(3.5, 2.0, 0.0), (0, 0, 0, 1)],
    [(1, 1.0, 0.0), (0, 0, 1, 0)],
    [(1, 1.0, 0.0), (0, 0, 1, 0)],
    [(3.5, 1.0, 0.0), (0, 0, 0, 1)],
    [(3.5, 1.0, 0.0), (0, 0, 0, 1)],
    [(1, 0.0, 0.0), (0, 0, 1, 0)],
    [(1, 0.0, 0.0), (0, 0, 1, 0)],
    [(3.5, 0.0, 0.0), (0, 0, 0, 1)],
    [(3.5, 0.0, 0.0), (0, 0, 0, 1)],
    [(1, -1.0, 0.0), (0, 0, 1, 0)],
    [(1, -1.0, 0.0), (0, 0, 1, 0)],
    [(3.5, -1.0, 0.0), (0, 0, 0, 1)],
    [(3.5, -1.0, 0.0), (0, 0, 0, 1)],
    [(1, -2.0, 0.0), (0, 0, 1, 0)],
    [(1, -2.0, 0.0), (0, 0, 1, 0)],
    [(3.5, -2.0, 0.0), (0, 0, 0, 1)],
    [(3.5, -2.0, 0.0), (0, 0, 0, 1)],
]
"""

 
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
        self.pose_status = 0
        self.explore = True
        self.done = False
        self.laser_status = 0
    def set_qr_status(self, status):
        self.qr_status = status

    def set_goal_status(self, status):
        self.goal_status = status

    def set_nxt_status(self, status):
        self.pose_status = status
        if status.position.x != -10.0:
            self.explore = False
        if status.position.x == 10:
            self.done = True

    #def set_laser_status(self, status):
        #self.laser_status = status

    def get_qr_status(self):
        return self.qr_status

    def get_goal_status(self):
        return self.goal_status

    def get_nxt_pose(self):
        return self.pose_status

    #def get_laser_status(self):
        #return self.laser_status.ranges[len(self.laser_status.ranges)/2]


def visp_callback(msg,callback_arg):
    callback_arg.set_qr_status(msg.data)

def pose_callback(msg, callback_arg):
    callback_arg.set_nxt_status(msg)

#def laser_callback(msg, callback_arg):
    #callback_arg.set_laser_status(msg.data)

def goal_callback(msg,callback_arg):
    status_list = msg.status_list
    length = len(status_list)
    if length > 0:
        callback_arg.set_goal_status(status_list[length-1].status)


if __name__ == '__main__':
    pub = rospy.Publisher("/flags_for_days",stm.Int8, queue_size = 1)
    rospy.init_node('patrol')

    wsp = Wasp()

    rospy.Subscriber("visp_auto_tracker/status",stm.Int8,visp_callback, wsp)
    rospy.Subscriber("move_base/status",alm.GoalStatusArray,goal_callback, wsp)
    rospy.Subscriber("nxt_loc",geo.Pose,pose_callback, wsp)
    #rospy.Subscriber("scan", laser.LaserScan, laser_callback, wsp)



    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()
    while not rospy.is_shutdown():
        while wsp.explore:
        
            for pose in waypoints:
                print "Sending a new goal!"   
                client.cancel_all_goals()
                goal = goal_pose(pose)
                client.send_goal(goal)
                goal_state = wsp.get_goal_status()
                print goal.target_pose.pose
                while goal_state != 3:
                    goal_state = wsp.get_goal_status()
                    qr_state = wsp.get_qr_status()
                    if qr_state > 2:
                        print "Is that a qr code???"
                        client.cancel_all_goals()
                        rospy.sleep(1)
                        pub.publish(1)
                        rospy.sleep(2)
                        pub.publish(0)
                        quat = (goal.target_pose.pose.orientation.x,goal.target_pose.pose.orientation.y,goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w)
                        eul = tft.euler_from_quaternion(quat)
                        quat = quaternion_from_euler(eul[0],eul[1],eul[2]-math.pi/4)
                        goal.target_pose.pose.orientation.x = quat[0]
                        goal.target_pose.pose.orientation.y = quat[1]
                        goal.target_pose.pose.orientation.z = quat[2]
                        goal.target_pose.pose.orientation.w = quat[3]
                        client.send_goal_and_wait(goal)
                        goal_state = 3


                if not wsp.explore:
                    print "Done Exploring"
                    print "=========================================="
                    print "Hidden frame calculated!"
                    print "=========================================="


                    break
                rospy.sleep(3)
                
        while not wsp.done:
            client.cancel_all_goals()
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'

            goal.target_pose.pose = wsp.get_nxt_pose()
            client.send_goal(goal)

            print "Navigation to next goal"
            print "=========================================="
            print goal.target_pose.pose
            print "=========================================="

            rospy.sleep(2)
            count = 0
            while goal.target_pose.pose.position == wsp.get_nxt_pose().position and count <= 8:
                goal_state = wsp.get_goal_status()
                qr_state = wsp.get_qr_status()

                if goal_state != 1 or qr_state > 2:
                    client.cancel_all_goals()
                    rospy.sleep(1)
                    pub.publish(1)
                    rospy.sleep(2)
                    pub.publish(0)
                    quat = (goal.target_pose.pose.orientation.x,goal.target_pose.pose.orientation.y,goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w)
                    eul = tft.euler_from_quaternion(quat)
                    quat = quaternion_from_euler(eul[0],eul[1],eul[2]-math.pi/8)
                    goal.target_pose.pose.orientation.x = quat[0]
                    goal.target_pose.pose.orientation.y = quat[1]
                    goal.target_pose.pose.orientation.z = quat[2]
                    goal.target_pose.pose.orientation.w = quat[3]
                    client.send_goal(goal)
                    print "Goal reached without reading qr. Turn!"
                    print "=========================================="
                    print goal.target_pose.pose
                    print "=========================================="
                count = count + 1

                rospy.sleep(1)

        print "Done"
    rospy.spin()

