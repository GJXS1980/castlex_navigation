#!/usr/bin/env python
#! -*- coding: utf-8 -*-
#from __future__ import unicode_literals


import roslib
#roslib.load_manifest('simple_navigation_goals_tutorial')
import rospy
import actionlib

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32MultiArray, Int64, Int32, Int32MultiArray


'''
##########################  主函数  ############################

'''
class MoveBaseDoor():
    def __init__(self):
        self.i, self.point, self.switch = 0, 0, 0
        self.cm = 0.0
        rospy.init_node('send_goals_node', anonymous=False)
        #rospy.on_shutdown(self.shutdown)

        # 机器人进行起始点位置的校准
        # self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        # self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # while self.cm <= 5000:
        #     move_cmd = Twist()
        #     if self.cm < 5000:
        #         move_cmd.angular.z = 1.0
        #         self.cmd_vel.publish(move_cmd) 
        #         self.cm += 0.01
        #         #time.sleep(50)
        #     else:
        #         move_cmd.angular.z = 0.0
        #         self.cmd_vel.publish(move_cmd) 
        #         self.cm += 0.01

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
            self.disinfect_pub.publish(self.switch)
            if self.i < 4 and self.point == 1:
                self.goal(self.i)
            else:
                self.switch = 0
                self.disinfect_pub.publish(self.switch)
                self.point = 0
                self.i = 0
        elif self.point == 0:
            self.switch = 0
            self.disinfect_pub.publish(self.switch)
        else:
            pass

    #   导航函数
    def goal(self, i):
        # 目标点的x,y,w坐标
        # waypointsx = list([4.58040380478, 4.60472297668, 4.72166395187, 5.39839839935])
        # waypointsy = list([3.54052853584, 4.80912876129, 5.95212841034, 6.07305908203])

        # waypointsaw = list([0.71244389762, 0.670867838495, 0.669185615067, -0.0749265288884])
        # waypointsw = list([0.701729073606, 0.741576930111, 0.743095291728, 0.997189056934])

        waypointsx = list([0.888956129551, 1.26235949993, -1.88215100765, -1.18901562691, 0.888956129551])
        waypointsy = list([0.684198081493, -0.381364881992, -0.216135337949, -1.17905819416, 0.684198081493])

        waypointsaw = list([0.186669080588, 0.155134544383, 0.986761009786, 0.980817486695, 0.186669080588])
        waypointsw = list([0.982422849059, 0.987893351096, -0.162181101138, -0.194928340151, 0.982422849059])

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
                self.i += 1
                #   发布导航成功的flag
                rospy.loginfo("You have reached the goal!")


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

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
