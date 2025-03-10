#!/usr/bin/env python
#coding=utf-8
import rospy
import string
import math
import time
import sys
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped,Pose


'''
MAIN FUCNTION
Multi goal_navigation
多点导航脚本+计时
'''
class MultiGoals:
    def __init__(self, goalListX, goalListY, retry, map_frame):
        self.retry=1
        if(self.retry==1):
            self.sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.statusCB, queue_size=10)
            self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        # params & variables
            self.goalListX = goalListX
            self.goalListY = goalListY
            self.goalListW = goalListW
            self.goalListZ = goalListZ
            self.flag=1
            self.goalId = 0
            self.count = 0
            self.goalMsg = PoseStamped()
            self.goalMsg.header.frame_id = map_frame
            time.sleep(1)
            self.goalMsg.header.stamp = rospy.Time.now()
            self.goalMsg.pose.position.x = self.goalListX[self.goalId]
            self.goalMsg.pose.position.y = self.goalListY[self.goalId]
            self.goalMsg.pose.orientation.z = self.goalListZ[self.goalId]
            self.goalMsg.pose.orientation.w = self.goalListW[self.goalId]
            self.pub.publish(self.goalMsg)
            self.start_time=rospy.get_time()
            rospy.loginfo("Initial goal published! Goal ID is: %d", self.goalId)
            self.goalId = self.goalId + 1

    def statusCB(self, data):
        #print(data.status.status)
        if data.status.status == 3 and self.flag==1: # reached and not the final goal
            finish_time = rospy.get_time()
            # interval=rospy.Time()
            interval = finish_time - self.start_time
            print(interval)
            self.goalMsg.header.stamp = rospy.Time.now()
            self.goalMsg.pose.position.x = self.goalListX[self.goalId]
            self.goalMsg.pose.position.y = self.goalListY[self.goalId]
            self.goalMsg.pose.orientation.z = self.goalListZ[self.goalId]
            self.goalMsg.pose.orientation.w = self.goalListW[self.goalId]
            self.pub.publish(self.goalMsg)
            rospy.loginfo("Initial goal published! Goal ID is: %d", self.goalId)
            rospy.loginfo("intostatusCB")
            self.count=self.count+1
            if self.count==len(self.goalListX):
                print("gg")
                self.flag=0
            if self.goalId < (len(self.goalListX)-1):
                self.goalId = self.goalId + 1

if __name__ == "__main__":
    try:
        # ROS Init
        rospy.init_node('multi_goals', anonymous=True)
        retry=1
        # Get params
        goalListX = rospy.get_param('~goalListX', '[ 6.3,13.0,2.0]')
        goalListY = rospy.get_param('~goalListY', '[ 0.2,0.0,2.0]')
        goalListZ = rospy.get_param('~goalListZ', '[ 0,0,1]')
        goalListW = rospy.get_param('~goalListW', '[0.9,1,0]')
        map_frame = rospy.get_param('~map_frame', 'map' )

        goalListX = goalListX.replace("[","").replace("]","")
        goalListY = goalListY.replace("[","").replace("]","")
        goalListZ = goalListZ.replace("[","").replace("]","")
        goalListW = goalListW.replace("[","").replace("]","")

        goalListX = [float(x) for x in goalListX.split(",")]
        goalListY = [float(y) for y in goalListY.split(",")]
        goalListZ = [float(z) for z in goalListZ.split(",")]
        goalListW = [float(w) for w in goalListW.split(",")]

        if len(goalListX) == len(goalListY) & len(goalListY) >=2 :
            # Constract MultiGoals Obj
            rospy.loginfo("Multi Goals Executing...")
            mg = MultiGoals(goalListX, goalListY, retry, map_frame)
            rospy.spin()
        else:
            rospy.loginfo("Lengths of goal lists are not the same")
    except KeyboardInterrupt:
        print("shutting down")
