#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from castlex_com_python import  CASTLEX_COM

if __name__ == '__main__':
    try:
        CASTLEX_COM()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("COM finished.")
