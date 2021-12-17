#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import time
#roslib.load_manifest('simple_navigation_goals_tutorial')
import rospy
import actionlib
from playsound import playsound

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32MultiArray, Int64
from std_msgs.msg import Float32MultiArray, Int32

'''
##########################  主函数  ############################

'''
class MoveBaseDoor():
    def __init__(self):
        self.i,self.dir,self.order,self.runing = 0,0,1,1  #[考点1] 起始点 方向:1为顺时针 0位逆时针 执行命令 执行状态 
        rospy.init_node('send_goals_node', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.id = 0

        # 机器人进行起始点位置的校准
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('/Commen_CMD_Topic', Int32, self.castlex_com_order)

        self.start = rospy.get_param("~start", "/voice/voice1.mp3")
        self.end = rospy.get_param("~end", "/voice/voice2.mp3")
        self.guest_rooms = rospy.get_param("~guest_rooms", "/voice/voice3.mp3")
        self.go_hall = rospy.get_param("~go_hall", "/voice/voice4.mp3")
        self.hall = rospy.get_param("~hall", "/voice/voice5.mp3")
        self.go_warehouse = rospy.get_param("~go_warehouse", "/voice/voice6.mp3")
        self.warehouse = rospy.get_param("~warehouse", "/voice/voice7.mp3")
        self.go_bedroom = rospy.get_param("~go_bedroom", "/voice/voice8.mp3")
        self.bedroom = rospy.get_param("~bedroom", "/voice/voice9.mp3")
        self.go_guest_rooms = rospy.get_param("~go_bedroom", "/voice/voice8.mp3")
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        playsound(self.start)
        rospy.sleep(1)
        self.rate = rospy.Rate(50)

        while self.runing and self.id < 4:
            if self.id ==0:
                self.routing(1, 2, 3)
            elif self.id ==1:
                self.routing(3, 4, 5)
            elif self.id ==2:
                self.routing(2, 6, 7)
            elif self.id ==3:
                self.routing(0, 8, 1)
            else:
                pass
            rospy.sleep(1)

    # 播放音频
    def play_mp3(self, data):
        if(data == 0) :
            playsound(self.start)
        elif data == 1:
            playsound(self.end)
        elif data == 2:
            playsound(self.guest_rooms) 
        elif data == 3:
            playsound(self.go_hall)
        elif data == 4:
            playsound(self.hall)
        elif data == 5:
            playsound(self.go_warehouse)
        elif data == 6:
            playsound(self.warehouse)
        elif data == 7:
            playsound(self.go_bedroom)
        elif data == 8:
            playsound(self.bedroom)
        elif data == 9:
            playsound(self.go_guest_rooms)
        else:
            pass
        rospy.sleep(1)
        

    # 路径规划
    def routing(self, data1, data2, data3):
        self.goal(data1)
        if self.data:
            self.play_mp3(data2)
            self.play_mp3(data3)
            self.data = False

    # 导航到目标点
    def castlex_com_order(self, data):
        if(data.data == 0 and self.order) :
            self.ac.cancel_goal()
        self.order = data.data
    #   导航函数
    def goal(self, i):
        # 目标点的x,y,w坐标(0:客房区,1:客厅,2:货仓,3:卧室)
 
        waypointsx = list([-1.63130521774, -2.0376663208, -0.697621107101, -0.0404697805643])
        waypointsy = list([0.622438073158, 0.249621331692, 0.165448963642, 0.40107691288])

        waypointsaw = list([0.996062953977, -0.750068449975, -0.464814701354, -0.153475837714])
        waypointsw = list([0.0886486983234, 0.661360204693, 0.885407981332, 0.988152400816])

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
                self.id += 1 
                self.data = True
                #   发布导航成功的flag
                rospy.loginfo("You have reached the goal!")


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.runing = 0 
        # Cancel any active goals
        #self.ac.cancel_goal()
        rospy.sleep(2)

if __name__ == '__main__':
    try:
        MoveBaseDoor()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
