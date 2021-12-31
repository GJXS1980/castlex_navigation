#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import unicode_literals
import rospy
from geometry_msgs.msg import Twist, Point
import time

from std_msgs.msg import Float32MultiArray, Int32, Float32, Int64
from sensor_msgs.msg import Joy


from playsound import playsound
import os

class CalibrateUlt():
    def __init__(self):
        # Give the node a name
        rospy.init_node('robot_escape_node', anonymous=False)
        self.grasp_flag, self.drop_flag = 0, 0

        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)
        self.sensor_pub = rospy.Publisher('/sensor_switch', Int32, queue_size = 1)

        # How fast will we check the odometry values?
        self.rate = 10
        r = rospy.Rate(self.rate)

        # 发布速度控制的话题
        # self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        # rospy.sleep(2)
        self.grasp_suc_flag = rospy.Publisher('/grasp_success_escape_flag', Int32, queue_size=5)
        self.drop_suc_flag = rospy.Publisher('/drop_success_escape_flag', Int32, queue_size=5)

        #   订阅传感器信息的话题
        # rospy.Subscriber("/sensor_data", Float32MultiArray, self.demonstration_play)
        rospy.Subscriber('/ultrasonic_data', Float32MultiArray, self.ultrasonic_data)

        rospy.Subscriber('/grasp_success', Int64, self.grasp_success_flag)
        rospy.Subscriber('/drop_success', Int32, self.drop_success_flag)

        rospy.spin()


    # 获取导航目标点的flag
    def grasp_success_flag(self, data):
        self.grasp_flag = data.data

    # 获取导航目标点的flag
    def drop_success_flag(self, data):
        self.drop_flag = data.data

    # 获取超声波数据
    def ultrasonic_data(self, data):
        self.gyro_left = data.data[1]
        self.gyro_right = data.data[0]
        if self.gyro_left == 0:
            sensor_switch = 7
            self.sensor_pub.publish(sensor_switch)
        else:
            pass
        if self.grasp_flag == 1:
            self.grasp_success_escape()
        elif self.drop_flag == 1:
            self.drop_success_escape()
        else:
            pass

    #   利用超声波的数据控制机器人远离工作台
    def grasp_success_escape(self):
        move_cmd = Twist()
        if self.gyro_left < 250:
            move_cmd.linear.x = -0.1
            move_cmd.angular.z = 0.0
            self.cmd_vel.publish(move_cmd) 

        else:
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.cmd_vel.publish(move_cmd) 
            self.grasp_num = 1
            self.grasp_suc_flag.publish(self.grasp_num)
            self.grasp_flag = 0

    #   利用超声波的数据控制机器人远离工作台
    def drop_success_escape(self):
        move_cmd = Twist()
        if self.gyro_left < 250:
            move_cmd.linear.x = -0.1
            move_cmd.angular.z = 0.0
            self.cmd_vel.publish(move_cmd) 

        else:
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.cmd_vel.publish(move_cmd) 
            self.drop_num = 1
            self.drop_suc_flag.publish(self.drop_num)
            self.drop_flag = 0

    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        # playsound(NULL)
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    CalibrateUlt()
    rospy.spin()
#    except:
#        rospy.loginfo("Calibration terminated.")
