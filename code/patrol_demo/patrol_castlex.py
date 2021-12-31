#!/usr/bin/env python
#! -*- coding: utf-8 -*-

import rospy
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from castlex_msgs.srv import *
from patrol_task_setup import *
from playsound import playsound

#   巡逻任务目标点的列表
task_list = {'living_room':['clean_living_room'], 'bedroom':['clean_bedroom'], 'kitchen':['clean_kitchen'], 'balcony':['clean_balcony']}

#   机器人停止任务
class Stop(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        pass
    def execute(self, userdata):
        rospy.loginfo("Shutting down the state machine")
        return 'succeeded'

#   客厅任务
class LivingRoom(State):
    def __init__(self, section, timer):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.task = 'clean_living_room'
        self.section = section
        self.timer = timer
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    def execute(self, userdata):
        rospy.loginfo('Cleaning living room in the ' + str(self.section))
        counter = self.timer
        # self.cmd_vel_pub.publish(Twist())
        message = "Finished cleaning living room the " + str(self.section) + "!"
        rospy.loginfo(message)
        # easygui.msgbox(message, title="Succeeded")
        # update_task_list(self.section, self.task)
        playsound("/home/castlex/castlex_ws/src/auto_demo/voice/params/livingroom.mp3")
        return 'succeeded'

#   卧室任务
class Bedroom(State):
    def __init__(self, section, timer):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.task = 'clean_bedroom'
        self.section = section
        self.timer = timer
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
   
    def execute(self, userdata):
        rospy.loginfo('Cleaning the bedroom in the ' + str(self.section))
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.05
        cmd_vel_msg.angular.z = 1.2
        counter = self.timer
        # self.cmd_vel_pub.publish(Twist())
        message = "Done cleaning the bedroom the " + str(self.section) + "!"
        rospy.loginfo(message)
        playsound("/home/castlex/castlex_ws/src/auto_demo/voice/params/bedroom.mp3")
        # easygui.msgbox(message, title="Succeeded")
        # update_task_list(self.section, self.task)
        return 'succeeded'

#   厨房任务
class Kitchen(State):
    def __init__(self, section, timer):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.task = 'clean_kitchen'
        self.section = section
        self.timer = timer
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    def execute(self, userdata):
        rospy.loginfo('Clean the kitchen...')
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.4
        cmd_vel_msg.angular.z = 0.2
        counter = self.timer
        # self.cmd_vel_pub.publish(Twist())
        message = "Done cleaning the kitchen!"
        rospy.loginfo(message)
        playsound("/home/castlex/castlex_ws/src/auto_demo/params/voice/kitchen.mp3")
        # easygui.msgbox(message, title="Succeeded")
        # update_task_list(self.section, self.task)
        return 'succeeded'

#   阳台任务
class Balcony(State):
    def __init__(self, section, timer):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.task = 'clean_balcony'
        self.section = section
        self.timer = timer
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    def execute(self, userdata):
        rospy.loginfo('Clean the balcony...')
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.4
        cmd_vel_msg.angular.z = 0.2
        counter = self.timer
        # self.cmd_vel_pub.publish(Twist())
        message = "Done cleaning the balcony!"
        rospy.loginfo(message)
        playsound("/home/castlex/castlex_ws/src/auto_demo/params/voice/bedroom.mp3")
        # easygui.msgbox(message, title="Succeeded")
        # update_task_list(self.section, self.task)
        return 'succeeded'
      
class Patrol_Smach():
    def __init__(self):
        #   初始化ROS节点
        rospy.init_node('patrol_task_concurrence', anonymous=False)
        # 设置关闭机器人函数(stop the robot)
        rospy.on_shutdown(self.shutdown)
        # 初始化一些参数和变量
        setup_task_environment(self)
        # 跟踪到达目标位置的成功率
        self.n_succeeded, self.n_aborted, self.n_preempted = 0, 0, 0
        # 保存上一个或者当前的导航目标点的变量
        self.last_nav_state = None
        # 指示是否正在充电的标志
        self.recharging = False
        # 保存导航目标点的列表
        nav_states = {}

        # 把waypoints变成状态机的状态
        for waypoint in self.room_locations.iterkeys():           
            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.frame_id = 'map'
            nav_goal.target_pose.pose = self.room_locations[waypoint]
            move_base_state = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb,
                                                 exec_timeout=rospy.Duration(180.0),
                                                 server_wait_timeout=rospy.Duration(10.0))
            # nav_states.append(move_base_state)
            nav_states[waypoint] = move_base_state
            
        # 为扩展底座（docking station）创建一个MoveBaseAction state
        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.header.frame_id = 'map'
        nav_goal.target_pose.pose = self.docking_station_pose
        nav_docking_station = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb,
                                             exec_timeout=rospy.Duration(180.0),
                                             server_wait_timeout=rospy.Duration(10.0))

        # 为living_room子任务创建一个状态机
        sm_living_room = StateMachine(outcomes=['succeeded','aborted','preempted'])
        # 然后添加子任务
        with sm_living_room:
            StateMachine.add('CLEAN_LIVING_ROOM', LivingRoom('living_room', 5), transitions={'succeeded':'','aborted':'','preempted':''})

        # 为bedroom子任务创建一个状态机
        sm_bedroom = StateMachine(outcomes=['succeeded','aborted','preempted'])
        # 然后添加子任务
        with sm_bedroom:
            StateMachine.add('CLEAN_BEDROOM', Bedroom('bedroom', 5), transitions={'succeeded':'','aborted':'','preempted':''})

        # 为kitchen子任务创建一个状态机
        sm_kitchen = StateMachine(outcomes=['succeeded','aborted','preempted'])
        # 然后添加子任务
        with sm_kitchen:
            StateMachine.add('CLEAN_KITCHEN', Kitchen('kitchen', 5), transitions={'succeeded':'','aborted':'','preempted':''})

        # 为balcony子任务创建一个状态机
        sm_balcony = StateMachine(outcomes=['succeeded','aborted','preempted'])
        # 然后添加子任务
        with sm_balcony:
            StateMachine.add('CLEAN_BALCONY', Balcony('balcony', 5), transitions={'succeeded':'','aborted':'','preempted':''})

        # 初始化巡逻导航的状态机
        self.sm_nav = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        
        # 使用transitions将导航的状态添加到状态机
        with self.sm_nav:
            #   创建巡逻任务开始状态的状态机
            StateMachine.add('START', nav_states['living_room'], transitions={'succeeded':'CLEAN_LIVING_ROOM','aborted':'CLEAN_LIVING_ROOM','preempted':''})
            
            ''' Add the living room subtask(s) '''
            StateMachine.add('CLEAN_LIVING_ROOM', nav_states['living_room'], transitions={'succeeded':'CLEAN_LIVING_ROOM_TASKS','aborted':'CLEAN_BEDROOM','preempted':''})
            #   当客厅导航任务完成时, 继续进行客厅环境检测任务
            StateMachine.add('CLEAN_LIVING_ROOM_TASKS', sm_living_room, transitions={'succeeded':'CLEAN_BEDROOM','aborted':'CLEAN_BEDROOM','preempted':''})
            
            ''' Add the bedroom subtask(s) '''
            StateMachine.add('CLEAN_BEDROOM', nav_states['bedroom'], transitions={'succeeded':'CLEAN_BEDROOM_TASKS','aborted':'CLEAN_KITCHEN','preempted':''})
            #   当卧室导航任务完成时, 继续进行卧室环境检测任务
            StateMachine.add('CLEAN_BEDROOM_TASKS', sm_bedroom, transitions={'succeeded':'CLEAN_KITCHEN','aborted':'CLEAN_KITCHEN','preempted':''})
            
            ''' Add the kitchen subtask(s) '''
            StateMachine.add('CLEAN_KITCHEN', nav_states['kitchen'], transitions={'succeeded':'CLEAN_KITCHEN_TASKS','aborted':'CLEAN_BALCONY','preempted':''})
            #   当厨房导航任务完成时, 继续进行厨房环境检测任务
            StateMachine.add('CLEAN_KITCHEN_TASKS', sm_kitchen, transitions={'succeeded':'CLEAN_BALCONY','aborted':'CLEAN_BALCONY','preempted':''})         
            
            ''' Add the balcony subtask(s) '''
            StateMachine.add('CLEAN_BALCONY', nav_states['balcony'], transitions={'succeeded':'CLEAN_BALCONY_TASKS','aborted':'START','preempted':''})
            #   当阳台导航任务完成时, 继续进行阳台环境检测任务
            StateMachine.add('CLEAN_BALCONY_TASKS', sm_balcony, transitions={'succeeded':'START','aborted':'START','preempted':''})                

        # 在sm_nav状态机中注册一个回调函数以启动状态转换（state transitions）
        self.sm_nav.register_transition_cb(self.nav_transition_cb, cb_args=[])

        # 初始化充电的状态机
        self.sm_recharge = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        
        with self.sm_recharge:
            StateMachine.add('NAV_DOCKING_STATION', nav_docking_station, transitions={'succeeded':'RECHARGE_BATTERY'})
            StateMachine.add('RECHARGE_BATTERY', ServiceState('battery_simulator/set_battery_level', SetBatteryLevel, 100, response_cb=self.recharge_cb), 
                             transitions={'succeeded':''})        

        # 使用并发容器（Concurrence container）创建nav_patrol状态机
        self.nav_patrol = Concurrence(outcomes=['succeeded', 'recharge', 'stop'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.concurrence_child_termination_cb,
                                        outcome_cb=self.concurrence_outcome_cb)
        
        # 将sm_nav machine和battery MonitorState添加到nav_patrol状态机里面         
        with self.nav_patrol:
           Concurrence.add('SM_NAV', self.sm_nav)
           Concurrence.add('MONITOR_BATTERY', MonitorState("battery_level", Float32, self.battery_cb))
        
        # 创建顶层状态机
        self.sm_top = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        
        # 将nav_patrol,sm_recharge和Stop添加到sm_top状态机
        with self.sm_top:
            StateMachine.add('PATROL', self.nav_patrol, transitions={'succeeded':'PATROL', 'recharge':'RECHARGE', 'stop':'STOP'}) 
            StateMachine.add('RECHARGE', self.sm_recharge, transitions={'succeeded':'PATROL'})
            StateMachine.add('STOP', Stop(), transitions={'succeeded':''})

        # 创建并开始SMACH introspection server
        intro_server = IntrospectionServer('patrol', self.sm_top, '/SM_ROOT')
        intro_server.start()
        
        # 运行状态机
        sm_outcome = self.sm_top.execute()
        
        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))
                
        intro_server.stop()
    
    def nav_transition_cb(self, userdata, active_states, *cb_args):
        self.last_nav_state = active_states
        
    # 当任何子状态终止时调用
    def concurrence_child_termination_cb(self, outcome_map):
        # 如果当前导航任务完成, return True
        if outcome_map['SM_NAV'] == 'succeeded':
            return True
        # 如果MonitorState状态变成False则储存当前的导航目标点并充电
        if outcome_map['MONITOR_BATTERY'] == 'invalid':
            rospy.loginfo("LOW BATTERY! NEED TO RECHARGE...")
            if self.last_nav_state is not None:
                self.sm_nav.set_initial_state(self.last_nav_state, UserData())
            return True
        else:
            return False
    
    # 当任何子状态终止时调用
    def concurrence_outcome_cb(self, outcome_map):
        # 如果电池电量低于设定的阈值,返回'recharge' outcome
        if outcome_map['MONITOR_BATTERY'] == 'invalid':
            return 'recharge'
        # 否则,如果最后一个导航目标点成功,返回'succeeded' 或者 'stop'
        elif outcome_map['SM_NAV'] == 'succeeded':
            self.patrol_count += 1
            rospy.loginfo("FINISHED PATROL LOOP: " + str(self.patrol_count))
            # 如果没有完成所有的巡逻，重新开始导航
            if self.n_patrols == -1 or self.patrol_count < self.n_patrols:
                # self.sm_nav.set_initial_state(['NAV_STATE_0'], UserData())
                return 'succeeded'
            # 否则,完成所有导航并返回 'stop'
            else:
                # self.sm_nav.set_initial_state(['NAV_STATE_4'], UserData())
                return 'stop'
        # 如果其他操作失败了，重新充电
        else:
            return 'recharge'
        
    def battery_cb(self, userdata, msg):
        if msg.data < self.low_battery_threshold:
            self.recharging = True
            return False
        else:
            self.recharging = False
            return True
        
    def recharge_cb(self, userdata, response):
        return 'succeeded'
        
    def move_base_result_cb(self, userdata, status, result):
        if not self.recharging:
            if status == actionlib.GoalStatus.SUCCEEDED:
                self.n_succeeded += 1
            elif status == actionlib.GoalStatus.ABORTED:
                self.n_aborted += 1
            elif status == actionlib.GoalStatus.PREEMPTED:
                self.n_preempted += 1
    
            try:
                rospy.loginfo("Success rate: " + str(100.0 * self.n_succeeded / (self.n_succeeded + self.n_aborted  + self.n_preempted)))
            except:
                pass

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        
        # self.sm_nav.request_preempt()
        
        # self.cmd_vel_pub.publish(Twist())
        
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Patrol_Smach()
    except rospy.ROSInterruptException:
        rospy.loginfo("SMACH test finished.")
