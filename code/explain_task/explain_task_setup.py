#!/usr/bin/env python
#! -*- coding: utf-8 -*-

import rospy
import actionlib
from actionlib import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import  pi
from collections import OrderedDict

from ruamel import yaml

def setup_task_environment(self):

    # 导入yaml文件
    data = (yaml.safe_load(open('/home/grantli/ws/src/auto_demo/params/explain_nav_waypoints.yaml'))) 

    # 设置低电量的阈值(between 0 and 100)
    self.low_battery_threshold = rospy.get_param('~low_battery_threshold', 30)
    
    # 设置巡逻的次数
    self.n_patrols = rospy.get_param("~n_patrols", 5)
    
    # 每个waypoint超时时间
    self.move_base_timeout = rospy.get_param("~move_base_timeout", 300) #seconds
    
    # 初始化巡逻计数器
    self.patrol_count = 0
    
    # 订阅move_base action server
    self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
    rospy.loginfo("Waiting for move_base action server...")
    
    # action server超时时间为60s
    self.move_base.wait_for_server(rospy.Duration(60))    
    
    rospy.loginfo("Connected to move_base action server")
    
    # # 创建列表保存目标的四元数(位姿)
    # quaternions = list()
    
    # # 定义目标方向角度（z轴方向）为欧拉角
    # euler_angles = (pi/2, pi, 3*pi/2, 0)
    
    # # 将角度变换成四元数
    # for angle in euler_angles:
    #     q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
    #     q = Quaternion(*q_angle)    #   转换成ROS下四元数的表达方式
    #     quaternions.append(q)
    
    # 创建一个列表来保存waypoint poses
    self.waypoints = list()
            
    # Append each of the four waypoints to the list.  Each waypoint
    # is a pose consisting of a position and orientation in the map frame.
    #   历史馆
    self.waypoints.append(yaml_fun(data, 'history_museum'))
    #   自然馆
    self.waypoints.append(yaml_fun(data, 'nature_museum'))
    #   陶瓷展览
    self.waypoints.append(yaml_fun(data, 'ceramics_exhibition'))
    #   端砚艺术展览
    self.waypoints.append(yaml_fun(data, 'duanyan_art_exhibition'))
    #  导引起始点
    self.waypoints.append(yaml_fun(data, 'init_position'))    
    # 创建展厅名名并映射到waypoint的位置
    room_locations = (('history_museum', self.waypoints[0]),
                      ('nature_museum', self.waypoints[1]),
                      ('ceramics_exhibition', self.waypoints[2]),
                      ('duanyan_art_exhibition', self.waypoints[3]),
                      ('init_position', self.waypoints[4]))
    
    # 将映射存储为有序字典，以便我们可以按顺序访问房间
    self.room_locations = OrderedDict(room_locations)
    
    # 扩展底座（docking station）的位置
    #   充电桩
    self.docking_station_pose = yaml_fun(data, 'charging_post')            
    
    # 初始化waypoint在RViz下可视化的标记
    init_waypoint_markers(self)
    
    # 设置每个waypoint在RViz下的可视化标记      
    for waypoint in self.waypoints:           
        p = Point()
        p = waypoint.position
        self.waypoint_markers.points.append(p)
        
    # 设置扩展底座（docking station）在RViz下的标记
    init_docking_station_marker(self)
        
    # 发布机器人控制指令 (e.g. to stop it)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    
    rospy.loginfo("Starting Tasks")
    
    # 发布waypoint markers
    self.marker_pub.publish(self.waypoint_markers)
    rospy.sleep(1)
    self.marker_pub.publish(self.waypoint_markers)
    
    # 发布docking station marker
    self.docking_station_marker_pub.publish(self.docking_station_marker)
    rospy.sleep(1)

def init_waypoint_markers(self):
    # Set up our waypoint markers
    marker_scale = 0.2
    marker_lifetime = 0 # 0 is forever
    marker_ns = 'waypoints'
    marker_id = 0
    marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
    
    # Define a marker publisher.
    self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=5)
    
    # Initialize the marker points list.
    self.waypoint_markers = Marker()
    self.waypoint_markers.ns = marker_ns
    self.waypoint_markers.id = marker_id
    self.waypoint_markers.type = Marker.CUBE_LIST
    self.waypoint_markers.action = Marker.ADD
    self.waypoint_markers.lifetime = rospy.Duration(marker_lifetime)
    self.waypoint_markers.scale.x = marker_scale
    self.waypoint_markers.scale.y = marker_scale
    self.waypoint_markers.color.r = marker_color['r']
    self.waypoint_markers.color.g = marker_color['g']
    self.waypoint_markers.color.b = marker_color['b']
    self.waypoint_markers.color.a = marker_color['a']
    
    self.waypoint_markers.header.frame_id = 'odom'
    self.waypoint_markers.header.stamp = rospy.Time.now()
    self.waypoint_markers.points = list()

def init_docking_station_marker(self):
    # Define a marker for the charging station
    marker_scale = 0.3
    marker_lifetime = 0 # 0 is forever
    marker_ns = 'waypoints'
    marker_id = 0
    marker_color = {'r': 0.7, 'g': 0.7, 'b': 0.0, 'a': 1.0}
    
    self.docking_station_marker_pub = rospy.Publisher('docking_station_marker', Marker, queue_size=5)
    
    self.docking_station_marker = Marker()
    self.docking_station_marker.ns = marker_ns
    self.docking_station_marker.id = marker_id
    self.docking_station_marker.type = Marker.CYLINDER
    self.docking_station_marker.action = Marker.ADD
    self.docking_station_marker.lifetime = rospy.Duration(marker_lifetime)
    self.docking_station_marker.scale.x = marker_scale
    self.docking_station_marker.scale.y = marker_scale
    self.docking_station_marker.scale.z = 0.02
    self.docking_station_marker.color.r = marker_color['r']
    self.docking_station_marker.color.g = marker_color['g']
    self.docking_station_marker.color.b = marker_color['b']
    self.docking_station_marker.color.a = marker_color['a']
    
    self.docking_station_marker.header.frame_id = 'odom'
    self.docking_station_marker.header.stamp = rospy.Time.now()
    self.docking_station_marker.pose = self.docking_station_pose

#   读取yaml文件中的目标点的位姿
def yaml_fun(data, str_word):
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
