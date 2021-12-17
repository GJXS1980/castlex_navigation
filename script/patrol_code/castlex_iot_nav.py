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

class MoveBaseDoor():
    def __init__(self):
        self.goal = 0 
        self.runing = 1
        self.target_point = -1
        self.back= 0
        self.success = 1

        rospy.init_node('send_goals_node', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('/Target_point_Topic', Int32, self.distribution_task)
        rospy.Subscriber('/Back_Ack_Topic', Int32, self.Back_Ack)
        rospy.Subscriber('/Warehouse_control', Int32, self.Warehouse_control)

        # 门铃控制,发送给物联网模块  1/0  开/关
        self.door_cmd = rospy.Publisher('/Door_CMD_Topic', Int32, queue_size=1)
        # 灯光控制,发送给物联网模块  1/0  开/关
        self.light_cmd = rospy.Publisher('/Lighting_CMD_Topic', Int32, queue_size=1)
        # 窗帘开关控制,发送给物联网模块  1/0  开/关
        self.trashcan_cmd = rospy.Publisher('/Trashcan_CMD_Topic', Int32, queue_size=1)
	
        self.voice1 = rospy.get_param("~file_path_a", "/voice/peisong_0.mp3")
        self.voice2 = rospy.get_param("~file_path_b", "/voice/peisong_1.mp3")
        self.voice3 = rospy.get_param("~file_path_c", "/voice/peisong_2.mp3")
        self.voice4 = rospy.get_param("~file_path_d", "/voice/peisong_3.mp3")

        self.rate = rospy.Rate(500)
        while self.runing :
            if(self.start) :

                    if self.success == 1 :              
                        forward_target_point(0,0,0,0)
                    elif self.success == 2 : 
                        forward_target_point(1,1,0,0)
#室房101 
                    if (self.goal == 3 and self.success  == 3) :
                       forward_target_point(3,0,0,0)
#室房102
                    elif (self.goal == 4 and self.success == 3) :
                       forward_target_point(4,0,0,0)
#室房103
                    elif (self.goal == 3 and self.success == 3) :
                       forward_target_point(5,0,0,0)
                    elif self.success == 3 : 
                        forward_target_point(2,0,0,0)
#返回
                    if self.success == 4  and self.back == 1: 
                       forward_target_point(2,1,0,0)
                    elif self.success == 5 :
                       forward_target_point(1,0,0,0)
                    elif self.success == 6 :
                        forward_target_point(0,0,0,0)
                        self.start = -1
                        self.goal = -1 
                        self.back= -1

            self.rate.sleep()
  
    # 导航到目标点
    def distribution_task(self, data):	
        if data.data == 2 :
            self.target_point = 3
        elif  data.data == 1 :
            self.target_point = 3     
        elif  data.data == 3 :
            self.target_point = 4   

        self.start = 1
        playsound(self.voice1)
 

   #开门
    def Warehouse_control(self, data):	
        if data.data == 1 :
            playsound(self.voice3)          

   # 导航返回
    def Back_Ack(self, data):	
        if data.data == 1 :
            self.back = 1

   #0 不播放
    def my_player(self,voice): 
        if 1:
            playsound(self.voice1)
	elif 2:
            playsound(self.voice2)
        elif 3:
            playsound(self.voice3)

    #路径规划 
    def forward_target_point(self,point,gateway_contr,trans_voice,task_voice): 
        if(gateway_contr == 1) :
            gateway_control(1)

        elif (gateway_contr == 0):
            gateway_control(0)  
        else :
            pass  
        
        my_player(trans_voice);
        send_goal(point)
        my_player(task_voice);
    # 导航到目标点
    def castlex_zwai_order(self, data): 
        if data.data == 0 :
             self.zwai_pub.publish(0)
        elif data.data == 1 :
             self.zwai_pub.publish(1)

        if(data.data == 0 and self.order) :
            self.ac.cancel_goal()
        else :
            rospy.sleep(3)
            playsound(self.voice1)
        self.order = data.data


    def gateway_control(self,action):
        outtime = 100
        if(action == 1):
            data = 0                     
                self.gateway_cmd.publish(1)                        
            while data != 1 and outtime > 0: 
                data = rospy.wait_for_messagee("/Gateway_State",Int32,timeout=50)
                self.gateway_cmd.publish(1)                   #申请开门禁     
                rospy.sleep(1) 
                outtime -=1       
        elif(action == 0):
            data = 1        
            self.gateway_cmd.publish(0)          
            while data != 0 and outtime > 0 :
                data = rospy.wait_for_messagee("/Gateway_State",Int32,timeout=50)   
                self.gateway_cmd.publish(0)                   #申请关门禁 
                rospy.sleep(1) 
                outtime -=1                 


    #   导航函数
    def send_goal(self, i):
        #目标点的x,y,w坐标(0:前台 1:闸门进 2:闸门出 3:客房101 4:卧室 5:客房102)
        waypointsx = list([ 0.524649977684, 0.649070858955, 1.14964079857, 1.94869840145,1.90166664124, 1.16113579273])
        waypointsy = list([ -0.0658102557063, -1.05558586121, -1.82821846008, -1.84693431854,-2.38284468651, -2.52348947525])
        waypointsaw = list([ -0.564307731792, -0.443226202751, -0.479166776264, 0.269536339287,-0.506848701428, 0.952902152588])
        waypointsw = list([ 0.825564524334, 0.896409802041, 0.877723874875, 0.962990218955,0.86203503053, -0.303277904888])

        # 订阅move_base服务器的消息
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.ac.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server");

        # 初始化goal为MoveBaseGoal类型
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
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
                self.success += 1 
                rospy.loginfo("You have reached the goal!")

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.runing = 0 
        self.ac.cancel_goal()
        rospy.sleep(2)

if __name__ == '__main__':
    try:
        MoveBaseDoor()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
