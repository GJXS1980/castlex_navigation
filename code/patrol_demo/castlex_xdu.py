#!/usr/bin/env python
#! -*- coding: utf-8 -*-
#from __future__ import unicode_literals

import roslib
#roslib.load_manifest('simple_navigation_goals_tutorial')
import rospy
import actionlib
import os

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32MultiArray, Int64, Int32, Int32MultiArray

from collections import OrderedDict
from ruamel import yaml

'''
##########################  主函数  ############################

'''
class MoveBaseDoor():
    def __init__(self):
        self.i, self.point, self.switch = 0, 0, 0
        self.cm = 0.0
        rospy.init_node('send_goals_node', anonymous=False)
        #rospy.on_shutdown(self.shutdown)

        # 导入yaml文件
        data = (yaml.safe_load(open('/home/castlex/castlex_ws/src/auto_demo/params/xdu_nav_waypoints.yaml'))) 

        # 创建一个列表来保存waypoint poses
        self.waypoints = list()
            
        # Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        #   客厅
        self.waypoints.append(self.yaml_fun(data, 'living_room'))
        #   卧室
        self.waypoints.append(self.yaml_fun(data, 'bedroom'))
        #   厨房
        self.waypoints.append(self.yaml_fun(data, 'kitchen'))
        #   阳台
        self.waypoints.append(self.yaml_fun(data, 'balcony'))
        #   客厅
        self.waypoints.append(self.yaml_fun(data, 'living_room'))
        print(len(self.waypoints))

        #   订阅陀螺仪数据作为一直在跑的一个线程
        rospy.Subscriber('/ultrasonic_data', Float32MultiArray, self.flag_data)

        #   订阅语音输入的命令词
        rospy.Subscriber('xdu_flag', Int32MultiArray, self.getGoalPoint)

        #   订阅是否开启消毒功能
        self.disinfect_pub = rospy.Publisher('disinfect_switch', Int32, queue_size = 1)

        rospy.spin()

    # #   获取语音输入的命令词
    def getGoalPoint(self,data):
        self.switch = data.data[0]
        self.point = data.data[1]
        # if self.point == 1:
        #     self.disinfect_pub.publish(self.switch)
        # else:
        #     pass


    # 导航到目标点
    def flag_data(self, data):
    	if self.point == 1:
            #self.disinfect_pub.publish(self.switch)
            if self.i < len(self.waypoints) and self.point == 1:
                self.goal(self.i)
            else:
                self.switch = 0
                self.disinfect_pub.publish(self.switch)
                self.point = 0
                self.i = 0
        #elif self.point == 0:
            #self.switch = 0
            #self.disinfect_pub.publish(self.switch)
        else:
            pass

    #   导航函数
    def goal(self, i):
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
        goal.target_pose.pose = self.waypoints[i]
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
            	self.playsound(self.i)

                self.i += 1
                #   发布导航成功的flag
                rospy.loginfo("You have reached the goal!")

    def playsound(self,num):
    	
    	audio_file = '/home/castlex/castlex_ws/src/auto_demo/res/' + str(num) + '.wav'
    	cmd = "play " + audio_file
        os.system(cmd)

    def shutdown(self):

        #self.switch = 0
        #self.disinfect_pub.publish(self.switch)
        rospy.loginfo("Stopping the robot...")

        # Cancel any active goals
        self.ac.cancel_goal()
        rospy.sleep(2)

        # Stop the robot
        #self.cmd_vel_pub.publish(Twist())
        #rospy.sleep(1)

    #   读取yaml文件中的目标点的位姿
    def yaml_fun(self, data, str_word):
        data = data.get(str_word)
        # 获取x,y,z位置
        pos_data = data['position']
        pos_x, pos_y, pos_z = pos_data.get('x'), pos_data.get('y'), pos_data.get('z')
        # 获取四元数
        ori_data = data['orientation']
        ori_x, ori_y, ori_z, ori_w = ori_data.get('x'), ori_data.get('y'), ori_data.get('z'), ori_data.get('w')
        #  转换成pose
        pose = Pose(Point(pos_x, pos_y, pos_z), Quaternion(ori_x, ori_y, ori_z, ori_w))
        return pose

if __name__ == '__main__':
    try:
        MoveBaseDoor()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
