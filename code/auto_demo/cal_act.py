#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import unicode_literals
import rospy
from geometry_msgs.msg import Twist, Point
import time

from std_msgs.msg import Float32MultiArray, Int32, Float32
from sensor_msgs.msg import Joy


from playsound import playsound
import os

class CalibrateUlt():
    def __init__(self):
        # Give the node a name
        rospy.init_node('ultrasonic_cal_node', anonymous=False)
        self.goal1_flag, self.goal2_flag = 0, 0

        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)
        self.sensor_pub = rospy.Publisher('/sensor_switch', Int32, queue_size = 1)

        # How fast will we check the odometry values?
        self.rate = 10
        r = rospy.Rate(self.rate)

        # 发布速度控制的话题
        # self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # rospy.sleep(2)
        self.grasp_cal_flag = rospy.Publisher('/grasp_cal_flag', Int32, queue_size=5)
        self.drop_cal_flag = rospy.Publisher('/drop_cal_flag', Int32, queue_size=5)

        self.rate = rospy.Rate(10)

        # while self.flag == 0:
        #   订阅传感器信息的话题
        # rospy.Subscriber("/sensor_data", Float32MultiArray, self.demonstration_play)
        rospy.Subscriber('/ultrasonic_data', Float32MultiArray, self.ultrasonic_data)
            # print(test)
        # time.sleep(1)

        rospy.Subscriber('/goal1_flag', Int32, self.goal1_flag_data)
        rospy.Subscriber('/goal2_flag', Int32, self.goal2_flag_data)
        while not rospy.is_shutdown():

            if self.goal1_flag == 1:
                self.grasp_cal()
            elif  self.goal2_flag == 1:
                self.drop_cal()
            else:
                pass
            self.rate.sleep()



    # 获取导航目标点的flag
    def goal1_flag_data(self, data):
        self.goal1_flag = data.data

    # 获取导航目标点的flag
    def goal2_flag_data(self, data):
        self.goal2_flag = data.data

    # 获取超声波数据
    def ultrasonic_data(self, data):

        self.gyro_left = data.data[1]
        self.gyro_right = data.data[0]
        if self.gyro_left == 0:
            sensor_switch = 7
            self.sensor_pub.publish(sensor_switch)
        else:
            pass
        

    #   利用两个超声波数据对机器人的位姿进行校准
    def grasp_cal(self):
        move_cmd = Twist()
        if (self.gyro_left - self.gyro_right) < -25:
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.25
            self.cmd_vel.publish(move_cmd) 

        elif (self.gyro_left - self.gyro_right) > 25:
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -0.25
            self.cmd_vel.publish(move_cmd) 

        else:
            #move_cmd.linear.x = 0.0
            #move_cmd.angular.z = 0.0
            #self.cmd_vel.publish(move_cmd) 
            if self.gyro_left > 120:
                move_cmd.linear.x = 0.2
                move_cmd.angular.z = 0.0
                self.cmd_vel.publish(move_cmd) 
            else:
                self.grasp_num = 1
                self.grasp_cal_flag.publish(self.grasp_num)
                self.goal1_flag = 0, 0
                self.cmd_vel.publish(Twist()) 

    #   利用两个超声波数据对机器人的位姿进行校准
    def drop_cal(self):
        move_cmd = Twist()
        if (self.gyro_left - self.gyro_right) < -25:
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.25
            self.cmd_vel.publish(move_cmd) 

        elif (self.gyro_left - self.gyro_right) > 25:
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -0.25
            self.cmd_vel.publish(move_cmd) 

        else:
            #move_cmd.linear.x = 0.0
            #move_cmd.angular.z = 0.0
            #self.cmd_vel.publish(move_cmd) 
            if self.gyro_left > 120:
                move_cmd.linear. x = 0.2
                move_cmd.angular.z = 0.0
                self.cmd_vel.publish(move_cmd) 
            else:
                self.drop_num = 1
                self.drop_cal_flag.publish(self.drop_num)
                self.goal2_flag = 0, 0
                self.cmd_vel.publish(Twist()) 


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
