#!/usr/bin/env python
#! -*- coding: utf-8 -*-
#from __future__ import unicode_literals

import roslib
#roslib.load_manifest('simple_navigation_goals_tutorial')
import rospy

from geometry_msgs.msg import Twist

from std_msgs.msg import Float32MultiArray, Int64, Int32, Int32MultiArray


'''
##########################  主函数  ############################

'''
class MoveBaseDoor():
    def __init__(self):
        self.i, self.point, self.switch = 0, 0, 0
        self.flag = 0
        self.cm = 0.0
        rospy.init_node('xdu_flag_node', anonymous=False)
        rospy.on_shutdown(self.shutdown)

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

    # 导航到目标点
    def flag_data(self, data):
        if self.point == 1 and self.switch != 0:
            self.disinfect_pub.publish(self.switch)
            self.switch = 0
            #self.point = 0
            self.flag = 1
        elif self.flag == 1 and self.switch != 0 and self.point == 0:
            self.switch = 0
            self.disinfect_pub.publish(self.switch)
            rospy.sleep(1)
            self.flag = 0
            # self.switch = 0
            # self.disinfect_pub.publish(self.switch)
        else:
            pass

    def shutdown(self):
        self.switch = 0
        self.disinfect_pub.publish(self.switch)
        rospy.sleep(2)
        rospy.loginfo("Stopping the robot...")

        

        # Stop the robot
        #self.cmd_vel_pub.publish(Twist())
        #rospy.sleep(1)


if __name__ == '__main__':
    try:
        MoveBaseDoor()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
