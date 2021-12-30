#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import time
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import Float32MultiArray, Int32, Float32
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

import paho.mqtt.client as mqtt
from collections import OrderedDict
import random
import ast
import json

from ruamel import yaml
# sudo pip install ruamel.yaml

class mqtt_arm():

    def __init__(self, ip, port):
        self.MQTTHOST = ip	# 服务器ip
        self.MQTTPORT = port	# 服务器端口
        # self.MQTTHOST = "192.168.10.3"	# 服务器ip
        # self.MQTTPORT = 50001	# 服务器端口
        self.mqttClient = mqtt.Client()
        #   Mqtt连接
        self.on_mqtt_connect()
        
        #self.on_subscribe()

        self.action = False
        self.castlex, self.arm = False, False
        self.sub_name, self.sub_dir, self.sub_action, self.sub_feedback, self.sub_msg = None, None, None, None, None
        #   初始化ros节点
        rospy.init_node('arm_node', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        #   订阅图像话题
        rospy.Subscriber('/usb_cam/image_raw', Image, self.mqtt_data)
        rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.mqtt_pub)
        rospy.Subscriber('/usb_cam/camera_info', CameraInfo, self.mqtt_subscribe)

        self.rate = rospy.Rate(50)
        rospy.spin()

    def mqtt_data(self, pose):
        if self.sub_action == "loading complete" and self.sub_feedback == 1:
            self.arm = False
        if self.sub_action == "loading" and self.sub_feedback == 0:
            self.castlex = True
        elif self.sub_action == "loading" and self.sub_feedback == 1:
            self.castlex = False
        if self.action:
            print("test")
            rospy.sleep(10)
            print("done")
            self.arm = True
            self.action = False

    def mqtt_subscribe(self, pose):
        #   订阅mqtt话题
        self.on_subscribe()

    #   发布Mqtt信息
    def mqtt_pub(self, data): 
        # print(self.sub_msg)
        if (self.castlex):
            self.on_publish("/castlex_arm_msg", self.arm_castlex_reply(), 0)
            self.action = True
            time.sleep(1)
        if self.arm:
            self.on_publish("/castlex_arm_msg", self.arm_castlex_cmd(), 0)
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
    def arm_castlex_reply(self):
        json_dict = OrderedDict()
        json_dict["name"] = "arm"
        json_dict["dir"] = "castlex"
        json_dict["ation"] = "loading"
        json_dict["error"] = "null"
        json_dict["feedback"] = 1
        param = json.dumps(json_dict)
        return param

    # 向机械臂回复收到装货完成指令
    def arm_castlex_cmd(self):
        json_dict = OrderedDict()
        json_dict["name"] = "arm"
        json_dict["dir"] = "castlex"
        json_dict["ation"] = "loading complete"
        json_dict["error"] = "null"
        json_dict["feedback"] = 0
        param = json.dumps(json_dict)
        return param

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.runing = 0 
        rospy.sleep(2)
        
# if __name__ == '__main__':
#     try:
#         mqtt_arm("192.168.3.10", 50001)
#         rospy.spin()

#     except rospy.ROSInterruptException:
#         rospy.loginfo("mqtt connect failed.")
