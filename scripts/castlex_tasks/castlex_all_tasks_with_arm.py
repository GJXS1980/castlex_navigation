#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from castlex_all_tasks_with_arm_python import castlex_with_arm

if __name__ == '__main__':
    try:
        castlex_with_arm()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("mqtt connect failed.")