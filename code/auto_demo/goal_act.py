#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
#roslib.load_manifest('simple_navigation_goals_tutorial')
import rospy
import actionlib
from ruamel import yaml
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32MultiArray, Int32, Float32, Int32MultiArray


'''
##########################  主函数  ############################

'''
class MoveBaseDoor():
    def __init__(self):
        self.i, self.start = 0, 0
        self.grasp_flag, self.drop_flag = 0, 0
        self.pointOne, self.pointTwo = -1, -1
        rospy.init_node('send_goals_node', anonymous=False)

        # 导入yaml文件
        data = (yaml.safe_load(open('/home/castlex/castlex_ws/src/auto_demo/params/auto_waypoints.yaml'))) 

        self.waypoints = list()
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
        
        self.rate = rospy.Rate(10)

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

        #   发布导航成功之后的flag
        self.goal1_pub = rospy.Publisher('/goal1_flag', Int32, queue_size = 1)
        self.goal2_pub = rospy.Publisher('/goal2_flag', Int32, queue_size = 1)

        #   订阅超聲波数据作为一直在跑的一个线程
        '''rospy.Subscriber('/ultrasonic_data', Float32MultiArray, self.flag_data)'''

        #   订阅超声波校准和机器人远离工作台成功之后的flag
        # rospy.Subscriber('/ultrasonic_cal_flag', Int32, self.cal_flag)


        rospy.Subscriber('/grasp_success_escape_flag', Int32, self.grasp_success_flag)
        rospy.Subscriber('/drop_success_escape_flag', Int32, self.drop_success_flag)

        #   订阅语音输入的命令词
        rospy.Subscriber('voice/goal_point', Int32MultiArray, self.getGoalPoint)

        while not rospy.is_shutdown():

            if self.pointOne != -1 and self.start == 0:
                self.goal(self.pointOne)
                self.i += 1
            elif self.pointTwo !=-1 and self.grasp_flag == 1:
                self.goal(self.pointTwo)
                self.start = 0
            self.rate.sleep()
            

    #   获取语音输入的命令词
    def getGoalPoint(self,data):
        self.pointOne = data.data[0]
        self.pointTwo = data.data[2]

    # # 获取超声波校准之后的flag
    # def cal_flag(self, data):
    #     self.cal_flag = data.data

    # 获取导航目标点的flag
    def grasp_success_flag(self, data):
        self.grasp_flag = data.data

    # 获取导航目标点的flag
    def drop_success_flag(self, data):
        self.drop_flag = data.data

    '''
    # 导航到目标点
    def flag_data(self, data):

        if self.start == 0 or self.drop_flag == 1:
        	self.i = 0
        	self.goal(self.pointOne)
        	self.i += 1
        elif self.grasp_flag == 1:
        	self.i = data.data
        	self.goal(self.pointTwo)
        else:
        	pass
    '''

    #   导航函数
    def goal(self, i):
        
        # 目标点的x,y,w坐标
        waypoint = self.waypoints[i]


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
        
        goal.target_pose.pose = waypoint
        rospy.loginfo("Sending goal")
        '''
        # 设置目标位置
        goal.target_pose.pose.position.x = waypointsx[i]
        goal.target_pose.pose.position.y = waypointsy[i]
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = waypointsaw[i]
        goal.target_pose.pose.orientation.w = waypointsw[i]
        
        '''
        # 机器人移动
        self.move(goal)


    def move(self, goal):
        self.ac.send_goal(goal)
        rospy.sleep(0.1)
        self.ac.send_goal(goal)

        # 设定5分钟的时间限制
        finished_within_time = self.ac.wait_for_result(rospy.Duration(300))

        # 如果5分钟之内没有到达，放弃目标
        if not finished_within_time:
            self.ac.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.ac.get_state()
            if state == GoalStatus.SUCCEEDED and self.i == 0:
                self.tag1, self.start, self.drop_flag, self.pointOne = 1, 1, 0, -1
                #   发布导航成功的flag
                self.goal1_pub.publish(self.tag1)
                rospy.loginfo("You have reached the goal!")

            elif state == GoalStatus.SUCCEEDED and self.i == 1:
                self.tag2, self.start, self.grasp_flag, self.pointTwo = 1, 1, 0, -1
                #   发布导航成功的flag
                self.goal2_pub.publish(self.tag2)
                rospy.loginfo("You have reached the goal!")

    def shutdown(self):
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
