#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from castlex_iot_spray_tasks_python import CASTLEX_IOT_SPRAY_TASKS
        
if __name__ == '__main__':
   try:
       CASTLEX_IOT_SPRAY_TASKS()
       rospy.spin()

   except rospy.ROSInterruptException:
       rospy.loginfo("Navigation finished.")
