#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from castlex_ultraviolet_tasks_python import CASTLEX_IOT_TASKS

if __name__ == '__main__':
    try:
        CASTLEX_IOT_TASKS()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("CASTLEX_IOT_TASKS failed.")
