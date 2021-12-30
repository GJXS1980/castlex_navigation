#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import time
import rospy
import actionlib
from playsound import playsound
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32MultiArray, Int64
from std_msgs.msg import Float32MultiArray, Int32

from ruamel import yaml
# sudo pip install ruamel.yaml
class MoveBaseDoor():
    def __init__(self):
        self.i, self.runing, self.id = 0, 1, 0  
        self.nav_data = False
        self.com_order = None
        #         初始化ros节点
        rospy.init_node('send_goals_node', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.routes, self.waypoints, self.sounds = [], list(), []
        #   导入yaml文件
        self.data = (yaml.safe_load(open('/home/castlex/castlex_ws/src/castlex_navigation/script/castlex_tasks/nav_waypoints.yaml'))) 

        #   发布紫外消杀话题
        self.ul_pub = rospy.Publisher("/ultraviolet_disinfection", Int32, queue_size=1)
        #   发布喷雾消杀话题
        self.sp_pub = rospy.Publisher("/spray_kill", Int32, queue_size=1)

        # 门铃控制,发送给物联网模块  1/0  开/关
        self.door_cmd = rospy.Publisher('/Door_CMD_Topic', Int32, queue_size=1)
        # 灯光控制,发送给物联网模块  1/0  开/关
        self.light_cmd = rospy.Publisher('/Lighting_CMD_Topic', Int32, queue_size=1)
        # 窗帘开关控制,发送给物联网模块  1/0  开/关
        self.trashcan_cmd = rospy.Publisher('/Trashcan_CMD_Topic', Int32, queue_size=1)
        # 门闸开关控制,发送给物联网模块  1/0  开/关
        self.gateway_cmd = rospy.Publisher('/Gateway_CMD_Topic', Int32, queue_size=1)


    #   获取yaml文件数据(data:yaml的文件路径，str_word:获取名称， i:提取几个导航点，j：选取的路径,k：路径有几个途经点 )
        self.routes = self.yaml_data(self.data, 'route', None, 4, 5, None)
        #   获取导航点
        self.waypoints = self.yaml_data(self.data, 'waypoint', 7, None, 5, None)
        #   获取语音文件
        self.sounds = self.yaml_data(self.data, 'sound', 4, None, None, 10)

        # 播放开始音频
        playsound(self.sounds[0])
        rospy.sleep(1)
        self.rate = rospy.Rate(50)
        self.test = 0
        while self.runing:
            if self.com_order == 1:
                # 用多少条路径
                self.routing_nav(5)
                if self.id == 4:
                    self.id = 0
                    self.test += 1
                if self.test == 1:
                    self.id = -1
          
    # 讲解话题命令词
    def voice_com_order(self, data):
        self.com_order = data.data

    # 结合路径和导航进行控制
    def routing_nav(self, data):
        for i in range(0, data):
            if self.id == i:
                con_data = eval(self.routes[i])
                self.routing(con_data[0], con_data[1], con_data[2])

    # 路径规划
    def routing(self, data, sound_data1, sound_data2):
        self.goal(data)
        if self.nav_data:
            for i in range(0, 10):
                if i == sound_data1:
                    playsound(self.sounds[sound_data1])
                    rospy.sleep(1)
                    break
                else:
                    pass
            for j in range(0, 10):
                if j == sound_data2:
                    playsound(self.sounds[sound_data2])
                    rospy.sleep(1) 
                    break
                else:
                    pass
            self.nav_data = False

    #   导航函数
    def goal(self, i):
        # 订阅move_base服务器的消息
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.ac.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")
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
                self.id += 1 
                self.nav_data = True
                #   发布导航成功的flag
                rospy.loginfo("You have reached the goal!")

    #   获取yaml文件数据(data:yaml的文件路径，str_word:获取名称， i:提取几个导航点，j：选取的路径,k：路径有几个途经点, sound:几个音频文件 )
    def yaml_data(self, yaml_data, str_word, i, j, k, sound):
        if str_word == 'waypoint':
            pose = list()
            for x in range(0, i):
                pose_name = str_word + str(x)
                data = yaml_data.get(pose_name)
                # 获取x,y,z位置
                pos_data = data['position']
                pos_x, pos_y, pos_z = pos_data.get('x'), pos_data.get('y'), pos_data.get('z')
                # 获取四元数
                ori_data = data['orientation']
                ori_x, ori_y, ori_z, ori_w = ori_data.get('x'), ori_data.get('y'), ori_data.get('z'), ori_data.get('w')
                #  转换成pose
                pose.append(Pose(Point(pos_x, pos_y, pos_z), Quaternion(ori_x, ori_y, ori_z, ori_w)))
            return pose
        elif str_word == 'route':
            station_data = []
            station_name = str_word + str(j)
            data = yaml_data.get(station_name)
            for y in range(0, k):
                route_data = 'station' + str(y)
                station_data.append(data.get(route_data))
            return station_data
        elif str_word == 'sound':
            sound_data = []
            data = yaml_data.get(str_word)
            for z in range(0, sound):
                sound_str_data = str_word + str(z)
                sound_data.append(data.get(sound_str_data))
            return sound_data

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
