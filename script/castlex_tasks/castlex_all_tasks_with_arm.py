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
from std_msgs.msg import Float32MultiArray, Int32, Float32
from nav_msgs.msg import Odometry
 
import paho.mqtt.client as mqtt
from collections import OrderedDict
import random
import ast
import json

from ruamel import yaml
# sudo pip install ruamel.yaml

class castlex_with_arm():
    def __init__(self):

        self.MQTTHOST = "192.168.10.2"	# 服务器ip
        self.MQTTPORT = 50001	# 服务器端口
        self.mqttClient = mqtt.Client()
        #   Mqtt连接
        self.on_mqtt_connect()
        #   订阅mqtt话题
        self.on_subscribe()

        self.i, self.runing, self.id = 0, 1, 0  
        self.nav_data, self.arm_data, self.castlex_data = False, False, False
        self.sub_msg, self.sub_name, self.sub_dir, self.sub_action, self.sub_feedback = None, None, None, None, None,
        self.odom_x, self.odom_y = 0.0, 0.0
        self.castlex_data = False
        self.back_id = 0

        #   初始化ros节点
        rospy.init_node('send_goals_node', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.routes, self.waypoints, self.sounds = [], list(), []
        #   导入yaml文件
        self.data = (yaml.safe_load(open('/home/castlex/castlex_ws/src/castlex_navigation/script/castlex_tasks/castlex_arm_waypoints.yaml'))) 


        rospy.Subscriber('/odom', Odometry, self.castlex_odom)
        rospy.Subscriber('/gyro_data', Float32, self.castlex_arm_mqtt)

        #   发布紫外消杀话题
        self.ul_pub = rospy.Publisher("/ultraviolet_disinfection", Int32, queue_size=1)
        #   发布喷雾消杀话题
        self.sp_pub = rospy.Publisher("/spray_kill", Int32, queue_size=1)
        #   发布货仓控制话题
        self.warehouse_pub = rospy.Publisher('/Warehouse_control', Int32, queue_size=1)
        # 门铃控制,发送给物联网模块  1/0  开/关
        self.door_cmd = rospy.Publisher('/Door_CMD_Topic', Int32, queue_size=1)
        # 灯光控制,发送给物联网模块  1/0  开/关
        self.light_cmd = rospy.Publisher('/Lighting_CMD_Topic', Int32, queue_size=1)
        # 窗帘开关控制,发送给物联网模块  1/0  开/关
        self.trashcan_cmd = rospy.Publisher('/Trashcan_CMD_Topic', Int32, queue_size=1)
        # 门闸开关控制,发送给物联网模块  1/0  开/关
        self.gateway_cmd = rospy.Publisher('/Gateway_CMD_Topic', Int32, queue_size=1)

    #   获取yaml文件数据(data:yaml的文件路径，str_word:获取名称， i:提取几个导航点，j：选取的路径,k：路径有几个途经点 )
        self.routes = self.yaml_data(self.data, 'route', None, 7, 3, None)
        #   获取导航点
        self.waypoints = self.yaml_data(self.data, 'waypoint', 8, None, 4, None)
        #   获取语音文件
        #self.sounds = self.yaml_data(self.data, 'sound', None, None, None, None)

        # 播放开始音频
        #playsound(self.sounds[0])
        #rospy.sleep(1)
        self.rate = rospy.Rate(50)
        while self.runing:
            # 用了几条路径，和self.routes第二个值对应上
            self.routing_iot_nav(3)
            rospy.spin()

    # 结合路径和导航进行控制
    def routing_iot_nav(self, data):
        for i in range(0, data):
            if self.id == i:
                con_data = eval(self.routes[i])
                self.iot_routing(con_data[0], con_data[1], con_data[2], con_data[3], con_data[4], con_data[5], con_data[6])

    # 路径规划(物联网) ， data: 导航点；warehouse_pub:货仓控制；arm_data: 机械臂控制；iot_light：物联网灯；iot_trashcan：物联网窗帘；iot_gateway：物联网闸机；iot_door：物联网门铃 
    def iot_routing(self, data, warehouse_data, arm_data, iot_light, iot_trashcan, iot_gateway,  iot_door):
        self.goal(data)
        if self.nav_data:
            #   货仓控制
            for i in range(0, 2):
                if i == warehouse_data:
                    self.warehouse_pub.publish(warehouse_data)
                    rospy.sleep(1)
                    break

            #   物联网灯
            for i in range(0, 8):
                if i == iot_light:
                    self.light_cmd.publish(iot_light)
                    rospy.sleep(1) 
                    break
            #   物联网窗帘
            for i in range(0, 2):
                if i == iot_trashcan:
                    self.trashcan_cmd.publish(iot_trashcan)
                    rospy.sleep(1) 
                    break

            #   物联网闸机
            for i in range(0, 2):
                if i == iot_gateway:
                    self.gateway_cmd.publish(iot_gateway)
                    rospy.sleep(1) 
                    break

            #   物联网门铃
            for i in range(0, 4):
                if i ==  iot_door:
                    self.door_cmd.publish(iot_door)
                    rospy.sleep(1) 
                    break

            #  发布以及到达装货点，请求装货
            for i in range(0, 2):
                if arm_data == 1:
                    #   发布mqtt话题
                    self.arm_data = True
                    #rospy.sleep(1)
                    break

            #   在等待装货完成信号
            while(arm_data):
                #   收到机械臂装货完成指令时，停止等待，否则一直在等待
                if self.sub_action == "loading complete" and self.sub_feedback == 1:
                    arm_data = 0
                    print("done")
                    # 关闭仓门
                    self.warehouse_pub.publish(0)
                    rospy.sleep(1)
                    break
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
        
    #   里程计控制函数
    def castlex_back_goal(self, data, data_vel): 
        move_cmd = Twist()
        while(self.back_id):
            if ((self.odom_back - self.odom_y) < data and (self.odom_back - self.odom_y) > 0) or ((self.odom_back - self.odom_y) > -(data) and (self.odom_back - self.odom_y) < 0):
                move_cmd.linear.x = data_vel
                self.cmd_vel.publish(move_cmd) 
            else:
                move_cmd.linear.x = 0.0
                self.cmd_vel.publish(move_cmd) 
                self.back_id = 0

    def castlex_odom(self, pose):
        self.odom_x = pose.pose.pose.position.x
        self.odom_y = pose.pose.pose.position.y
        if self.sub_action == "loading" and self.sub_feedback == 1:
            self.arm_data = False
        if self.sub_action == "loading complete" and self.sub_feedback == 0:
            self.castlex_data = True
        elif self.sub_action == "loading complete" and self.sub_feedback == 1:
            self.castlex_data = False

    #   发布Mqtt信息
    def castlex_arm_mqtt(self, data): 
        if (self.arm_data):
            self.on_publish("/castlex_arm_msg", self.castlex_arm_cmd(), 0)
            time.sleep(1)
        if self.castlex_data:
            self.on_publish("/castlex_arm_msg", self.castlex_arm_reply(), 0)
            time.sleep(1)
        
    # 连接MQTT服务器
    def on_mqtt_connect(self):
        self.mqttClient.connect(self.MQTTHOST, self.MQTTPORT, 60)
        self.mqttClient.loop_start()

    # publish mqtt 消息
    def on_publish(self, topic, payload, qos):
        #print("pub msg: %s to %s" % (payload, topic))
        self.mqttClient.publish(topic, payload, qos)

    # 对mqtt订阅消息处理函数
    def on_message_come(self, lient, userdata, msg):
	# python3 bytes转str
        self.sub_msg = str(msg.payload, 'utf-8')
	    # python3 str转字典
        self.sub_msg = ast.literal_eval(self.sub_msg)
        if len(self.sub_msg) == 5:
            self.sub_name, self.sub_dir, self.sub_action, self.sub_feedback = self.sub_msg['name'], self.sub_msg['dir'], self.sub_msg['ation'], self.sub_msg['feedback']
        else:
            pass

    # mqtt subscribe 消息
    def on_subscribe(self):
        self.mqttClient.subscribe("/castlex_arm_msg", 0)
        self.mqttClient.on_message = self.on_message_come  # 消息到来处理函数

    # 向机械臂发送请求装货指令
    def castlex_arm_cmd(self):
        json_dict = OrderedDict()
        json_dict["name"] = "castlex"
        json_dict["dir"] = "arm"
        json_dict["ation"] = "loading"
        json_dict["error"] = "null"
        json_dict["feedback"] = 0
        param = json.dumps(json_dict)
        return param

    # 向机械臂回复收到装货完成指令
    def castlex_arm_reply(self):
        json_dict = OrderedDict()
        json_dict["name"] = "castlex"
        json_dict["dir"] = "arm"
        json_dict["ation"] = "loading complete"
        json_dict["error"] = "null"
        json_dict["feedback"] = 1
        param = json.dumps(json_dict)
        return param

if __name__ == '__main__':
    try:
        castlex_with_arm()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
