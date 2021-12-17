#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
#roslib.load_manifest('simple_navigation_goals_tutorial')
import rospy
import actionlib
from playsound import playsound

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32MultiArray, Int32


'''
##########################  主函数  ############################

'''
class MoveBaseDoor():
    def __init__(self):
        self.i, self.cm,self.runing,self.order = 0,0.0 ,1,0
        rospy.init_node('send_goals_node', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # 机器人进行起始点位置的校准
        # self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('/Disinfect_CMD_Topic', Int32, self.castlex_xdu_order)
        self.xdu_pub = rospy.Publisher("/disinfect_switch", Int32, queue_size=1)
        # 灯光控制,发送给物联网模块  1/0  开/关
        self.light_cmd = rospy.Publisher('/Lighting_CMD_Topic', Int32, queue_size=1)

        self.voice0 = rospy.get_param("~file_path_s", "/voice/voice0.mp3")
        self.voice1 = rospy.get_param("~file_path_a", "/voice/voice1.mp3")
        self.voice2 = rospy.get_param("~file_path_b", "/voice/voice2.mp3")
        self.voice3 = rospy.get_param("~file_path_c", "/voice/voice3.mp3")
        self.voice4 = rospy.get_param("~file_path_d", "/voice/voice4.mp3")
        self.voice5 = rospy.get_param("~file_path_e", "/voice/voice5.mp3")
        self.voice6 = rospy.get_param("~file_path_f", "/voice/voice6.mp3")
        self.voice7 = rospy.get_param("~file_path_g", "/voice/voice7.mp3")
        self.voice8 = rospy.get_param("~file_path_h", "/voice/voice8.mp3")
        self.voice9 = rospy.get_param("~file_path_i", "/voice/voice9.mp3")
        self.voice10 = rospy.get_param("~file_path_j", "/voice/voice10.mp3")
        self.voice11 = rospy.get_param("~file_path_k", "/voice/voice11.mp3")
        playsound(self.voice0)

        #   订阅语音输入的命令词
        # rospy.Subscriber('nav_position', Int64, self.getGoalPoint)
        self.rate = rospy.Rate(50)
        while self.runing:
            if(self.order) :
                if  self.i > 3 :
                     self.i = 0 

                rospy.sleep(2)
                if self.i == 0 :
                    playsound(self.voice1)   
                elif self.i == 1 :
                    playsound(self.voice3) 
                    self.light_cmd.publish(2)  
                elif self.i == 2 :
                    playsound(self.voice5)  
                elif self.i == 3 :
                    playsound(self.voice7)  

                self.goal(self.i)
                self.i += 1
            self.rate.sleep()
        #rospy.spin()

    # #   获取语音输入的命令词
    # def getGoalPoint(self,data):
    #     self.point = data.data

    # 导航到目标点
    def castlex_xdu_order(self, data):
        if data.data == 0:
            self.xdu_pub.publish(0)
        elif data.data == 1 :
            self.xdu_pub.publish(1)
        elif data.data == 2 :
            self.xdu_pub.publish(2)
        elif data.data == 3 :
            self.xdu_pub.publish(3)
        elif data.data == 4 :
            self.xdu_pub.publish(2)
	
        self.order = data.data
        if(self.order == 0) :
            self.ac.cancel_goal()
        else :
            rospy.sleep(3)
    # 导航到目标点
    def flag_data(self, data):
        if self.i == 4:
        	self.i = 0
        	self.goal(self.i)
        else:
        	self.goal(self.i)

    #   导航函数
    def goal(self, i):
        # 目标点的x,y,w坐标(0:客房区,1:客厅,2:货仓,3:卧室)
 
         
        waypointsx = list([-0.556773424149,1.55975556374,0.472913384438,-1.54197978973])
        waypointsy = list([-0.261611044407,0.916353166103,3.26766633987,1.89096283913])

        waypointsaw = list([0.29690269947,0.717356556001,0.95695806399, 0.305900095203])
        waypointsw = list([0.954907737453,0.69670623046,-0.290226228594,0.952063617494])
        # 订阅move_base服务器的消息
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        self.ac.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server");

        # 初始化goal为MoveBaseGoal类型
        goal = MoveBaseGoal()

        # 使用map的frame定义goal的frame id
        goal.target_pose.header.frame_id = 'map'

        # 设置时间戳
        goal.target_pose.header.stamp = rospy.Time.now()

        # 设置目标位置
        goal.target_pose.pose.position.x = waypointsx[i]
        goal.target_pose.pose.position.y = waypointsy[i]
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = waypointsaw[i]
        goal.target_pose.pose.orientation.w = waypointsw[i]
        rospy.loginfo("Sending goal")
        # 机器人移动
        self.move(goal)


    def move(self, goal):
        self.ac.send_goal(goal)

        # 设定5分钟的时间限制
        finished_within_time = self.ac.wait_for_result(rospy.Duration(300))

        # 如果5分钟之内没有到达，放弃目标
        if not finished_within_time:
            self.ac.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.ac.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.sleep(2)
                if self.i == 0 :
                    playsound(self.voice2)   
                elif self.i == 1 :
                    playsound(self.voice4)  
                elif self.i == 2 :
                    playsound(self.voice6)
                    self.light_cmd.publish(0)   
                elif self.i == 3 :
                    playsound(self.voice8)
                    self.order = 0
                    self.xdu_pub.publish(0)
                    rospy.sleep(1) 
                    playsound(self.voice9) 
                #   发布导航成功的flag
                rospy.loginfo("You have reached the goal!")


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.runing = 0
        # Cancel any active goals
        self.ac.cancel_goal()
        rospy.sleep(2)

        # Stop the robot
        #self.cmd_vel_pub.publish(Twist())
        #rospy.sleep(1)



if __name__ == '__main__':
    try:
        MoveBaseDoor()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
