#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
#roslib.load_manifest('simple_navigation_goals_tutorial')
import rospy
import actionlib

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32MultiArray, Int64


'''
##########################  主函数  ############################

'''
class MoveBaseDoor():
    def __init__(self):
        self.i, self.cm,self.runing,self.dir = -1, 0.0 ,1,0
        rospy.init_node('send_goals_node', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # 机器人进行起始点位置的校准
        # self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # while self.cm <= 7000:
        #     move_cmd = Twist()
        #     if self.cm < 7000:
        #         move_cmd.angular.z = 0.5
        #         self.cmd_vel.publish(move_cmd) 
        #         self.cm += 0.01
        #         #time.sleep(50)
        #     else:
        #         move_cmd.angular.z = 0.0
        #         self.cmd_vel.publish(move_cmd) 
        #         self.cm += 0.01


        #   订阅陀螺仪数据作为一直在跑的一个线程
        #rospy.Subscriber('/ultrasonic_data', Float32MultiArray, self.flag_data)

        #   订阅语音输入的命令词
        # rospy.Subscriber('nav_position', Int64, self.getGoalPoint)
        self.rate = rospy.Rate(50) 
        while self.runing:
            if self.i >= 3:
                self.dir = 1                   
            elif self.i <= 0 :
                self.dir = 0                     

            if(self.dir):
                self.i-=1
            else :	
                self.i+=1
                #print (self.i)
            self.goal(self.i)

        self.rate.sleep()
        #rospy.spin()

    # #   获取语音输入的命令词
    # def getGoalPoint(self,data):
    #     self.point = data.data


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

        waypointsx = list([-0.773351073265,0.0352573171258,1.01714038849,2.06956481934])
        waypointsy = list([-0.142344489694,-0.224107950926,-0.279988646507,-0.315526425838])

        waypointsaw = list([0.00995563474153,-0.0303931400301,-0.0528727430333,0.999254508051])
        waypointsw = list([0.99995044144,0.999538021808,0.998601258283,0.0386060635065])
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
                #self.i += 1
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
