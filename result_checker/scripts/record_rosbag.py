#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys,os,time
import rospy, rospkg
import rosbag
from morai_msgs.msg import ObjectInfo, EgoVehicleStatus, ScenarioLoad, ReplayInfo
from std_msgs.msg import String



class rosbag_record():
    def __init__(self):
        rospy.init_node('rosbag_recorder',anonymous=True)
        
        # initialize
        self.ego_msg=EgoVehicleStatus()
        self.object_msg=ObjectInfo()
        self.file_name = ""
        self.record_start = False
        
        # subscriber
        rospy.Subscriber("/rosbag_start", String, self.rosbagCB) ## rosbag 시작 알림 및 파일명 전송
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.egoCB) ## 기록할 토픽
        rospy.Subscriber('/Object_topic',ObjectInfo, self.objectCB) ## 기록할 토픽
        print("Before wait")
        # WAIT (시작 알림을 받고, 시나리오도 불러왔을 때)
        while(len(self.file_name) <= 0):
            # print("wait")
            pass
        
        f_path = '/home/ubuntu/Desktop/MORAI/cloud_scenario_manager/data/bag/' + self.file_name + ".bag"
        print(self.file_name)
        self.bag = rosbag.Bag(f_path,'w')
        self.record_start = True
        # RECORD
        while not rospy.is_shutdown():
            pass
        self.bag.close()


    
    def rosbagCB(self,data):
        self.file_name = data.data

    def egoCB(self,data):
        self.ego_msg=data
        if(self.record_start):
            self.bag.write("/Ego_topic", self.ego_msg)

    def objectCB(self,data):
        self.object_msg=data
        if(self.record_start):
            self.bag.write("/Object_topic", self.object_msg)

    # def scCB(self,data):
    #     self.file_name=data.file_name

if __name__ == '__main__':
    try:
        sc_strt =rosbag_record()
    finally :
        # QUIT
        print("bag file saved")
        # sc_strt.bag.close()
