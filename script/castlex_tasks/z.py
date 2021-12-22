#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time

from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import Float32, Int64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import ast

'''
##########################  主函数  ############################

'''
class Odom_xy():
    def __init__(self):
	# 初始化数据
        self.data1_sta, self.odom_x, self.odom_y, self.data2_sta =  1, 0.0, 0.0, 0
        rospy.init_node('odom_node', anonymous=False)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.sub = rospy.Subscriber('/agv_nav_point', Int64 , self.nav_id) 
        rospy.Subscriber('/odom', Odometry, self.castlex_odom)
        rospy.Subscriber('/gyro_data', Float32, self.odom_cmd)

        rospy.spin()

    def castlex_odom(self, data):
        self.data_l = data.data[0]
        self.data_r = data.data[1]

    def nav_id(self, data):
        self.x_id_result = data.data

    def odom_cmd(self, data):
        self.odom_xy(0.3, 0.3)

    #   函数
    def odom_xy(self, data1, data2): 
        move_cmd = Twist()
        while(self.data1_sta):
            if (self.odom_x < data1):
                move_cmd.linear.x = 0.05
                self.cmd_vel.publish(move_cmd)                
            else:
                move_cmd.linear.x = 0.0
                self.cmd_vel.publish(move_cmd) 
                self.data1_sta, self.data2_sta = 0, 1
       
        while(self.data2_sta):
            if (self.odom_y < data2):
                move_cmd.linear.y = 0.05
                self.cmd_vel.publish(move_cmd)                
            else:
                move_cmd.linear.y = 0.0
                self.cmd_vel.publish(move_cmd) 
                self.data2_sta = 0

if __name__ == '__main__':
    try:
        Odom_xy()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Odom_xy finished.")
