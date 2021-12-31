#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from castlex_nav_python import CASTLEX_NAV

if __name__ == '__main__':
    try:
        CASTLEX_NAV()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("CASTLEX_NAV failed.")
