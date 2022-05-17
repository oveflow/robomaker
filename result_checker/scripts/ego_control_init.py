#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy,os,sys, time
import rospkg
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus,CollisionData, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv 
class EgoInitController:
    def __init__(self):

        ctrl_cmd = CtrlCmd()
        self.ego = EgoVehicleStatus()
        rospy.init_node('init_controller', anonymous=True)
        cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)

        rospy.wait_for_service('Service_MoraiEventCmd')
        event_mode_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_callback) ## Vehicl Status Subscriber 

        # AutoMode & gear P
        event_cmd = EventInfo()
        event_cmd.option = 3
        event_cmd.ctrl_mode = 3 # automode
        event_cmd.gear = 4 # P
        event_resp = event_mode_srv(event_cmd)

        # time.sleep(0.5)
        
        rate = rospy.Rate(30)
        ctrl_cmd.longlCmdType = 1
        ctrl_cmd.brake = 1

        for i in range(10):
            cmd_pub.publish(ctrl_cmd)
            rate.sleep()
        event_resp = event_mode_srv(event_cmd)


        exit(1)


    def ego_callback(self,data):
        self.ego = data # EgoVehicleStatus()
if __name__ == '__main__':
    try:
        ego_init =EgoInitController()
    except rospy.ROSInterruptException:
        pass