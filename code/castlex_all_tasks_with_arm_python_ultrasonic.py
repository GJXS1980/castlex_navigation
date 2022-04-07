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
from std_msgs.msg import Float32MultiArray, Int32, Float32, String, Int32MultiArray
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
        self.iot_data = Int32MultiArray()
        self.i, self.runing, self.id = 0, 1, 0  
        self.nav_data, self.arm_data, self.castlex_data = False, False, False
        self.sub_msg, self.sub_name_str, self.sub_name, self.sub_dir, self.sub_action, self.sub_feedback = None, None, None, None, None, None,
        self.odom_x, self.odom_y = 0.0, 0.0
        self.castlex_data, self.ack_data_flag = False, False
        # self.odom_front, self.odom_back = 0, 0
        self.front_id, self.back_id = 1, 1
        self.ultrasonic_data_l, self.ultrasonic_data_r = 0.0, 0.0
        self.front_flag, self.back_flag = False, False

        #   初始化ros节点
        rospy.init_node('castlex_arm_all_tasks_node', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.routes, self.waypoints, self.sounds = [], list(), []
        #   导入yaml文件
        self.yaml_path = rospy.get_param("~yaml_path", '/home/castlex/castlex_ws/src/castlex_navigation/script/castlex_tasks/nav_waypoints.yaml')
        self.data = (yaml.safe_load(open(self.yaml_path))) 

        #   导航路径参数设置
        self.routes_j = rospy.get_param("~routes_j", 4) #   选择第几条路径
        self.routes_k = rospy.get_param("~routes_k", 4) #   路径有几个途经点

        #   导航点参数设置
        self.waypoints_nav = rospy.get_param("~waypoints_nav", 8) #   导航点

        #   mqtt服务器参数设置
        self.MQTTHOST = rospy.get_param("~MQTTHOST", "192.168.3.10") #   服务器ip
        self.MQTTPORT = rospy.get_param("~MQTTPORT", 50001) #   服务器端口
        self.mqttClient = mqtt.Client()
        #   Mqtt连接
        self.on_mqtt_connect()
        #   订阅mqtt话题
        self.on_subscribe()

        #   超声波校准参数设置
        self.front_dis = rospy.get_param("~front_dis", 95) #   停止距离
        self.back_dis = rospy.get_param("~back_dis", 200) #    逃逸距离

        self.front_action_time = rospy.get_param("~front_action_time", 5) #    前进动作时间
        self.back_action_time = rospy.get_param("~back_action_time", 5) #    后退动作时间

        self.front_vel = rospy.get_param("~front_vel", 0.1)  #   靠近速度
        self.back_vel = rospy.get_param("~back_vel", -0.1)  #   远离速度

        rospy.Subscriber('/odom', Odometry, self.castlex_odom)
        rospy.Subscriber('/gyro_data', Float32, self.castlex_arm_mqtt)

        self.one_data, self.two_data, self.fruit_one_data, self.fruit_two_data, self.ack_data = 0, 0, 0, 0, None
        #   订阅货仓输入目标点1指令
        rospy.Subscriber('/ADDREES_ONE', Int32, self.position_one)
        #   订阅货仓输入目标点2指令
        rospy.Subscriber('/ADDREES_TWO', Int32, self.position_two)
        #   订阅货仓输入水果1指令
        rospy.Subscriber('/Fruit_Select_ONE', Int32, self.fruit_one)
        #   订阅货仓输入水果2指令
        rospy.Subscriber('/Fruit_Select_TWO', Int32, self.fruit_two)
        #   订阅配送完成状态
        rospy.Subscriber('/Back_Ack_Topic', Int32, self.ack_status)

        #   超声波
        rospy.Subscriber('/ultrasonic_data', Float32MultiArray, self.ultrasonic_data)

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
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        # 物联网模块
        self.iot_data_pub = rospy.Publisher("/iot_control", Int32MultiArray, queue_size=1)

        #   打开超声波
        self.mqtt_msg_pub = rospy.Publisher("/mqtt_msg", String, queue_size=1)

        #   获取yaml文件数据(data:yaml的文件路径，str_word:获取名称， i:提取几个导航点，j：选取的路径,k：路径有几个途经点 )
        self.routes = self.yaml_data(self.data, 'route', None, self.routes_j, self.routes_k, None)
        #   获取导航点
        self.waypoints = self.yaml_data(self.data, 'waypoint', self.waypoints_nav, None, None, None)
        #   获取语音文件
        #self.sounds = self.yaml_data(self.data, 'sound', None, None, None, None)

        # 播放开始音频
        #playsound(self.sounds[0])
        #rospy.sleep(1)
        #   打开所有传感器
        # self.sensor_pub.publish(7)
        # time.sleep(3)

        while self.runing:
            time.sleep(0.1)
            # 在货仓设置目标点和水果时才启动
            if self.one_data != 0:
                # 用了几条路径，和self.routes第二个值对应上
                self.routing_iot_nav(self.routes_k)
            else:
                pass
        #rospy.spin()

    #   目标点1
    def position_one(self, data): 
        self.one_data = data.data
    #   目标点2
    def position_two(self, data): 
        self.two_data  = data.data
    #   水果1
    def fruit_one(self, data): 
        self.fruit_one_data  = data.data
    #   水果2
    def fruit_two(self, data): 
        self.fruit_two_data  = data.data
    #   状态反馈
    def ack_status(self, data): 
        self.ack_data  = data.data
        #print(self.ack_data)
        if self.ack_data == 1 or self.ack_data == 0:
            self.ack_data_flag = True

    def ultrasonic_data(self, data):
        self.ultrasonic_data_l = data.data[0]
        self.ultrasonic_data_r = data.data[1]
        if self.ultrasonic_data_l > self.ultrasonic_data_r:
            self.ultrasonic_odom_data = self.ultrasonic_data_r
        else:
            self.ultrasonic_odom_data = self.ultrasonic_data_l
        # print(self.ultrasonic_odom_data, self.front_flag)
        if self.front_flag:
            self.castlex_odom_front(self.front_dis, self.front_vel)
        elif self.back_flag:
            self.castlex_odom_back(self.back_dis, self.back_vel)
    # 结合路径和导航进行控制
    def routing_iot_nav(self, data):
        for i in range(0, data):
            if self.id == i:
                con_data = eval(self.routes[i])
                self.iot_routing(con_data[0], con_data[1], con_data[2], con_data[3], con_data[4])

    # 路径规划(物联网) ， data: 导航点；warehouse_pub:货仓控制；arm_data: 机械臂控制；iot_data：物联网模块； warehouse_ack_data：取完货货仓控制） 
    def iot_routing(self, data, warehouse_data, arm_data, iot_data, warehouse_ack_data):
        self.goal(data)
        if self.nav_data:
            #   货仓控制
            for i in range(0, 2):
                if i == warehouse_data:
                    self.warehouse_pub.publish(warehouse_data)
                    rospy.sleep(0.1)
                    break
            #   物联网模块控制
            if iot_data[1] == 0 or iot_data[1] == 1:
                self.iot_data.data = iot_data
                self.iot_data_pub.publish(self.iot_data)

            #  发布以及到达装货点，请求装货
            for i in range(0, 2):
                if arm_data == 1:
                    #   前进
                    self.front_flag = True
                    rospy.sleep(self.front_action_time) 
                    # self.castlex_odom_front(self.front_dis, self.front_vel)
                    #   发布mqtt话题
                    # self.arm_data = True
                    #rospy.sleep(1)
                    break

            #   在等待装货完成信号
            while(arm_data):
                time.sleep(0.1)
                #   收到机械臂装货完成指令时，停止等待，否则一直在等待
                if self.sub_action == "loading complete" and self.sub_feedback == 1:
                    arm_data = 0
                    print("done")
                    # 关闭仓门
                    self.warehouse_pub.publish(0)
                    rospy.sleep(0.5)
                    # 后退
                    self.back_flag = True
                    rospy.sleep(self.back_action_time) 
                    # self.castlex_odom_back(self.back_dis, self.back_vel)
                    break
            
            #   投递时货仓控制
            if (not warehouse_ack_data):
                self.nav_data = False
            elif warehouse_ack_data:
                while(warehouse_ack_data):
                    if self.ack_data_flag:
                        #print("test1")
                        # 关仓门
                        self.warehouse_pub.publish(0)
                        rospy.sleep(0.1)
                        self.nav_data = False
                        self.ack_data = None
                        self.ack_data_flag = False
                        warehouse_ack_data = 0
                    else:
                        rospy.Subscriber('/Back_Ack_Topic', Int32, self.ack_status)
                        pass
                

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
                # self.odom_front = self.odom_x
                
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
    def castlex_odom_front(self, data, data_vel): 
        move_cmd = Twist()
        if self.front_id:
            if self.ultrasonic_odom_data > data:
                move_cmd.linear.x = data_vel
                self.cmd_vel.publish(move_cmd) 
            else:
                self.arm_data = True
                move_cmd.linear.x = 0.0
                self.cmd_vel.publish(move_cmd) 
                self.front_id = 0
                self.front_flag = False

    #   里程计控制函数
    def castlex_odom_back(self, data, data_vel): 
        move_cmd = Twist()
        if self.back_id:
            if self.ultrasonic_odom_data < data:
                move_cmd.linear.x = data_vel
                self.cmd_vel.publish(move_cmd) 
            else:
                move_cmd.linear.x = 0.0
                self.cmd_vel.publish(move_cmd) 
                self.back_id = 0
                self.back_flag = False

    def castlex_odom(self, pose):
        self.odom_x = pose.pose.pose.position.x
        self.odom_y = pose.pose.pose.position.y
        if self.sub_action == "loading" and self.sub_feedback == 1:
            self.arm_data = False
        if self.sub_action == "loading complete" and self.sub_feedback == 0:
            self.castlex_data = True
        elif self.sub_action == "loading complete" and self.sub_feedback == 1:
            self.castlex_data = False
        # print(self.sub_action, self.sub_feedback)

    #   发布Mqtt信息
    def castlex_arm_mqtt(self, data): 
        if self.arm_data:
            self.on_publish("/castlex_arm_msg", self.castlex_arm_cmd(), 0)
            time.sleep(1)
        if self.castlex_data:
            self.on_publish("/castlex_arm_msg", self.castlex_arm_reply(), 0)
            time.sleep(1)
        self.mqtt_msg_pub.publish(self.sub_name_str)
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
        self.sub_msg_str = str(msg.payload, 'utf-8')
	    # python3 str转字典
        self.sub_msg = ast.literal_eval(self.sub_msg_str)
        print(self.sub_msg)
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
        json_dict["object"] = [self.fruit_one_data, self.fruit_two_data]
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
        # json_dict["object"] = [self.fruit_one_data, self.fruit_two_data]
        json_dict["feedback"] = 1
        param = json.dumps(json_dict)
        return param

# if __name__ == '__main__':
#     try:
#         castlex_with_arm()
#         rospy.spin()

#     except rospy.ROSInterruptException:
#         rospy.loginfo("Navigation finished.")
