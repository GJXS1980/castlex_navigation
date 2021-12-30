#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from arm_mqtt_ros_python import mqtt_arm

if __name__ == '__main__':
    try:
        mqtt_arm("192.168.3.10", 50001)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("mqtt connect failed.")