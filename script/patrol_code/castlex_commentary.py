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
from std_msgs.msg import Float32MultiArray, Int64
from std_msgs.msg import Float32MultiArray, Int32

'''
##########################  主函数  ############################

'''
class MoveBaseDoor():
    def __init__(self):
        self.i,self.dir,self.order,self.runing = 0,0,0,1  #[考点1] 起始点 方向:1为顺时针 0位逆时针 执行命令 执行状态 
        rospy.init_node('send_goals_node', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.routing = 0
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

        if (self.i == 0 and self.dir == 0) :
            self.routing = 1  
        elif (self.i == 1  and  self.dir == 0) :
            self.routing = 2
        elif (self.i == 1 and  self.dir == 1) :
            self.routing = 3
        elif (self.i == 2  and self.dir == 0) :
            self.routing = 4       
        elif (self.i == 2  and self.dir == 1) :
            self.routing = 5
        elif (self.i == 3  and self.dir == 0) :
            self.routing = 6

        print(self.routing )
        rospy.sleep(5)
        playsound(self.start)
        rospy.sleep(1)
        self.rate = rospy.Rate(50)
        while self.runing :
            if(self.order) :
                self.goal(self.i)

                if self.i >= 3:
                    self.dir = 1                   
                elif self.i <= 0 :
                    self.dir = 0                     

                if(self.dir):
                    self.i-=1
                else :	
                    self.i+=1
                #print (self.i)
	#    rospy.spinOnce() 
        self.rate.sleep()

    # 导航到目标点
    def castlex_com_order(self, data):
        if(data.data == 0 and self.order) :
            self.ac.cancel_goal()
        self.order = data.data
    #   导航函数
    def goal(self, i):
        # 目标点的x,y,w坐标(0:客房区,1:客厅,2:货仓,3:卧室)
 
        waypointsx = list([-0.773351073265,0.0352573171258,1.01714038849,2.06956481934])
        waypointsy = list([-0.142344489694,-0.224107950926,-0.279988646507,-0.315526425838])

        waypointsaw = list([0.00995563474153,-0.0303931400301,-0.0528727430333,0.999254508051])
        waypointsw = list([0.99995044144,0.999538021808,0.998601258283,0.0386060635065])
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # 订阅move_base服务器的消息
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
s
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
                if (self.routing == 1 ) :  #路线1 
                    if(self.i == 0 and self.dir==0) :
                        playsound(self.guest_rooms) #[考点2] 播放相关音频
                        rospy.sleep(1)
                        playsound(self.go_hall)
                    elif(self.i == 1 and self.dir==0 ) :
                        playsound(self.hall)
                        rospy.sleep(2)
                        playsound(self.go_warehouse)
                        rospy.sleep(1)
                    elif(self.i == 2 and self.dir==0 ) :
                        playsound(self.warehouse)
                        rospy.sleep(2)
                        playsound(self.go_bedroom)
                        rospy.sleep(1)
                    elif(self.i == 3 and self.dir==0 ) :
                        playsound(self.bedroom)
                        rospy.sleep(2)
                        playsound(self.end)
                        rospy.sleep(1)
                        self.order = 0 

                elif (self.routing == 2 ) : #路线2 
                    if(self.i == 1 and self.dir==0) :
                        playsound(self.hall)
                        rospy.sleep(1)
                        playsound(self.go_warehouse)                       
                    elif(self.i == 2 and self.dir==0 ) :
                        playsound(self.warehouse)
                        rospy.sleep(2)
                        playsound(self.go_bedroom)
                        rospy.sleep(1)
                    elif(self.i == 3 and self.dir==0 ) :
                        playsound(self.bedroom)
                        rospy.sleep(2)
                        playsound(self.go_guest_rooms)
                        rospy.sleep(1)
                    elif(self.i == 0 and self.dir==1) :
                        playsound(self.guest_rooms)
                        rospy.sleep(2)
                        playsound(self.end)
                        rospy.sleep(1)
                        self.order = 0

                elif (self.routing == 3 ) : #路线3 
                    if(self.i == 1 and self.dir==1) :
                        playsound(self.hall)
                        rospy.sleep(1)
                        playsound(self.go_guest_rooms)                       
                    elif(self.i == 0 and self.dir==1 ) :
                        playsound(self.guest_rooms)
                        rospy.sleep(2)
                        playsound(self.go_warehouse)
                        rospy.sleep(1)
                    elif(self.i == 2 and self.dir==0 ) :
                        playsound(self.warehouse)
                        rospy.sleep(2)
                        playsound(self.go_bedroom)
                        rospy.sleep(1)
                    elif(self.i == 3 and self.dir==0) :
                        playsound(self.bedroom)
                        rospy.sleep(2)
                        playsound(self.end)
                        rospy.sleep(1)
                        self.order = 0

                elif (self.routing == 4 ) : #路线4
                    if(self.i == 2 and self.dir==0) :
                        playsound(self.warehouse)
                        rospy.sleep(1)
                        playsound(self.go_bedroom)
                    elif(self.i == 3 and self.dir==0 ) :
                        playsound(self.bedroom)
                        rospy.sleep(2)
                        playsound(self.go_hall)
                        rospy.sleep(1)
                    elif(self.i == 1 and self.dir== 1 ) :
                        playsound(self.hall)
                        rospy.sleep(2)
                        playsound(self.go_guest_rooms)   
                        rospy.sleep(1)
                    elif(self.i == 0 and self.dir==1 ) :
                        playsound(self.guest_rooms)
                        rospy.sleep(2)
                        playsound(self.end)
                        rospy.sleep(1)
                        self.order = 0 

                elif (self.routing == 5 ) : #路线5
                    if(self.i == 2 and self.dir==1) :
                        playsound(self.warehouse)
                        rospy.sleep(1)
                        playsound(self.go_hall)
                    elif(self.i == 1 and self.dir==1 ) :
                        playsound(self.hall)
                        rospy.sleep(2)
                        playsound(self.go_guest_rooms) 
                        rospy.sleep(1)
                    elif(self.i == 0 and self.dir==1 ) :
                        playsound(self.guest_rooms)
                        rospy.sleep(2)
                        playsound(self.go_bedroom)
                        rospy.sleep(1)
                    elif(self.i == 3 and self.dir==0 ) :
                        playsound(self.bedroom)
                        rospy.sleep(2)
                        playsound(self.end)
                        self.order = 0

                elif (self.routing == 6 ) : #路线6
                    if(self.i == 3 and self.dir== 0) :
                        playsound(self.bedroom)
                        rospy.sleep(1)
                        playsound(self.go_warehouse)
                    elif(self.i == 2 and self.dir==1 ) :
                        playsound(self.warehouse)
                        rospy.sleep(2)
                        playsound(self.go_hall)
                        rospy.sleep(1)
                    elif(self.i == 1 and self.dir==1) :
                        playsound(self.hall)
                        rospy.sleep(2)
                        playsound(self.go_guest_rooms) 
                        rospy.sleep(1)
                    elif(self.i == 0 and self.dir==1) :
                        playsound(self.guest_rooms)
                        rospy.sleep(2)
                        playsound(self.end)
                        self.order = 0    
                #   发布导航成功的flag
                rospy.loginfo("You have reached the goal!")


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.runing = 0 
        # Cancel any active goals
        self.ac.cancel_goal()
        rospy.sleep(2)

if __name__ == '__main__':
    try:
        MoveBaseDoor()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
