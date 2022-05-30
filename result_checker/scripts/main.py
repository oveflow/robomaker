#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, rospkg,rosbag
import os,sys,json
import numpy as np
import cv2
import time,datetime

from std_msgs.msg import Float64,String
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
from morai_msgs.msg import  EgoVehicleStatus,CollisionData ,ReplayInfo , SaveSensorData

from math import pow
from collections import deque

from s3_uploader import upload_to_aws

class WriteData:

    def __init__(self, pkg_name , scenario_file , result_save_dir):
        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path(pkg_name)
        
        curr_date = datetime.datetime.now()
        self.folder_name = result_save_dir
        
        self.dir_path = pkg_path +'/data/'+self.folder_name
        self.createFolder()

        self.txt_path=self.dir_path + '/' + scenario_file +'.txt'
        self.txt_f=open(self.txt_path, 'w')

        self.vehicle_path=self.dir_path + '/' + scenario_file +'_vehicle.mp4'
        self.vehicle_f=open(self.vehicle_path, 'w')

        self.top_path=self.dir_path + '/' + scenario_file +'_top.mp4'
        self.top_f=open(self.top_path, 'w')

        self.bag_path = self.dir_path + '/' + scenario_file +'.bag'
        self.bag = rosbag.Bag(self.bag_path,'w')
        
        self.video_vehicle_path = self.dir_path + '/' + scenario_file +'_vehicle.mp4' #mp4
        self.video_vehicle_tmp_path = self.dir_path + '/' + scenario_file +'_vehicle_tmp.mp4'
        self.video_vehicle_view = cv2.VideoWriter(self.video_vehicle_tmp_path,cv2.VideoWriter_fourcc(*'mp4v'),20,(640,480))#mp4v                  
                                    #file_path,     file_format  ,                 fps, size

        self.change_code_h264_cmd_vehicle = "ffmpeg -i " + self.video_vehicle_tmp_path +" -vcodec libx264 " + self.video_vehicle_path


        self.video_top_path = self.dir_path + '/' + scenario_file +'_top.mp4' #mp4
        self.video_top_tmp_path = self.dir_path + '/' + scenario_file +'_top_tmp.mp4'
        self.video_top_view = cv2.VideoWriter(self.video_top_tmp_path,cv2.VideoWriter_fourcc(*'mp4v'),20,(640,480))#mp4v                  
                                    #file_path,     file_format  ,                 fps, size

        self.change_code_h264_cmd_top = "ffmpeg -i " + self.video_top_tmp_path +" -vcodec libx264 " + self.video_top_path


    def createFolder(self):
        try:
            if not os.path.exists(self.dir_path):
                os.makedirs(self.dir_path)
        except OSError:
            print ('Error: Creating directory. ' +  self.dir_path)

    def temp_video_data(self,data):
        self.vehicle_f.write(data)
        self.top_f.write(data)
    
    
    def write_txt(self , data):
        self.txt_f.write(data)

    def close_txt(self):
        self.txt_f.close()

    def write_bag(self, msg):
        self.bag.write("/ReplayInfo_topic", msg)

    def close_bag(self):
        self.bag.close()

    def write_video(self,vehicle_view_data,top_view_data):
        for i in range(len(vehicle_view_data)):
            self.video_vehicle_view.write(vehicle_view_data[i])
        self.video_vehicle_view.release()        

        for i in range(len(top_view_data)):
            self.video_top_view.write(top_view_data[i])
        self.video_top_view.release()
        
        os.system(self.change_code_h264_cmd_vehicle)
        os.system(self.change_code_h264_cmd_top)

class ResultChecker:
    def __init__(self):

        self.ego = EgoVehicleStatus()
        self.max_collision = 3
        self.collision_count = 0
        self.is_stop_start = False
        arg = rospy.myargv(argv=sys.argv)


        #task_id
        self.result_save_dir = arg[1]
        
        #scenario_id
        self.scenario_file = arg[2]
        self.result_record_file_name = self.scenario_file

        #map_name        
        self.map_name = arg[3]
        

        start_time = self.getCurrTimeStr()
        self.start_time_secs = time.time()

        print(self.map_name,self.scenario_file,start_time)

        self.write_data = WriteData('result_checker', self.result_record_file_name , self.result_save_dir)

        self.write_data.write_txt("MAP : " + self.map_name + "\n")
        self.write_data.write_txt("scenario_file_name : " + self.scenario_file + "\n")
        self.write_data.write_txt("start_time : " + start_time + "\n")
        
        self.is_bag_record = True

        rospy.init_node('result_checker', anonymous=True)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_callback) ## Vehicl Status Subscriber 
        rospy.Subscriber("/CollisionData", CollisionData, self.collision_callback) ## Object information Subscriber
        rospy.Subscriber("/ReplayInfo_topic", ReplayInfo, self.rosbag_replay_callback) ## rosbag 시작 알림 및 파일명 전송
        rospy.Subscriber("/image_jpeg_vehicle_view/compressed",CompressedImage, self.img_vehicle_callback)
        rospy.Subscriber("/image_jpeg_top_view/compressed",CompressedImage, self.img_top_callback)
        rospy.Subscriber("/done_sig",String,self.done_sig_callback)
        self.pub = rospy.Publisher('/dist', Float64, queue_size=1)

        self.result_pub = rospy.Publisher('/result_signal', String, queue_size=1)

        #for videio capture
        self.vehicle_count = 0 
        self.vehicle_img_array = deque()
        self.is_vehicle_img = False
        self.is_done = False

        self.top_count = 0
        self.top_img_array = deque()        
        self.is_top_img = False
        self.is_saving = False

        while not rospy.is_shutdown():
            self.result_pub.publish('result_play')
            # self.check_ego_drive()
            if self.is_collision():
                self.save_result_data("Fail (Collision)")
                break

            if self.is_goalin():
                self.save_result_data("Success")
                print("Success")
                break
            
            if self.is_stop():            
                self.save_result_data("Vehicle_is_stop_5_sec") 
                break
            

    def check_ego_drive(self):

        if self.is_collision():
            self.save_result_data("Fail (Collision)")
            exit(0) # Fail

        if self.is_goalin():
            self.save_result_data("Success")
            print("Success")
            exit(0) # Success
        
        if self.is_stop():            
            self.save_result_data("Vehicle_is_stop_5_sec") 
            exit(0)
    

    def save_result_data(self, is_success):
        self.is_bag_record = False
        play_time = time.time() - self.start_time_secs
        end_time = self.getCurrTimeStr()
        self.write_data.write_txt("end_time : " + end_time + "\n")
        self.write_data.write_txt("total_run_time : " + str(play_time) + "\n")
        self.write_data.write_txt("result : " + is_success +"\n")
        self.write_data.close_txt()
        self.write_data.close_bag()
        
        

        # upload_to_aws(self.scenario_file_path, 'cloud-scenario-data', self.result_save_dir + '/' + self.scenario_file + '.json')
        upload_to_aws(self.write_data.txt_path, 'morai-sim-output', self.result_save_dir + '/' + self.result_record_file_name +'.txt')
        upload_to_aws(self.write_data.bag_path, 'morai-sim-output', self.result_save_dir + '/' + self.result_record_file_name +'.bag')        
        print("is_success:{0}".format(is_success))
        print("Vehicle_is_stop_5_sec")
        if is_success == "Fail (Collision)" or is_success == "Fail (No Driving)" or is_success == "Vehicle_is_stop_5_sec": 
            print("save_mp4_data")
            self.is_saving = True 
            self.write_data.write_video(self.vehicle_img_array,self.top_img_array)     
            #for demo temp mp4 data
            # self.write_data.temp_video_data(is_success)
            upload_to_aws(self.write_data.video_vehicle_path, 'morai-sim-output', self.result_save_dir + '/' + self.result_record_file_name +'_vehicle_view.mp4')
            upload_to_aws(self.write_data.video_top_path, 'morai-sim-output', self.result_save_dir + '/' + self.result_record_file_name +'_top_view.mp4')
        
        


    def is_collision(self):
        if(self.collision_count > self.max_collision):
            return True
        else:
            return False

    def is_goalin(self):
        self.destination_list = [[-10.101255,1057.39209], #KATRI
                                [8.14135742188,11.7146606445],#Sangam
                                [-1036.8972168,4906.19287109],#brt0                                
                                [217.620117188,386.432128906],#junction
                                [479.415893555,439.630859375]]#suburb
        min_dis = float('inf')
        for destation in self.destination_list:
            
            dist = pow((destation[0] - self.ego.position.x) , 2) + pow((destation[1] - self.ego.position.y), 2)

            if min_dis > dist :
                min_dis = dist
                
        self.pub.publish(min_dis)
        if( min_dis < pow(5, 2 )):
            return True
        else:
            return False

    def is_stop(self):
            
        if(self.ego.velocity.x < 1):
            if(not self.is_stop_start):
                self.is_stop_start = True
                self.is_stop_time = time.time()
            else:
                if(time.time() - self.is_stop_time > 5):
                    return True
        else:
            self.is_stop_start = False

        return False


    ## callback ## 

    def ego_callback(self,ego_msg):
        self.ego = ego_msg

    def collision_callback(self,collsion_msg): 
        if(len(collsion_msg.collision_object) > 0):
            if(self.collision_count > self.max_collision):
                self.collision_count = 0
            else:
                self.collision_count += 1


    def rosbag_replay_callback(self, replay_msg):
        if(self.is_bag_record):
            self.write_data.write_bag(replay_msg)


    def getCurrTimeStr(self):
        curr_time = datetime.datetime.now()

        curr_time_str = str(curr_time.month) + \
                        str(curr_time.day)   + \
                        str(curr_time.hour)  + \
                        str(curr_time.minute)+ \
                        str(curr_time.second)

        return curr_time_str

    def img_vehicle_callback(self, img_msg):
        if not self.is_saving  :
            np_arr = np.fromstring(img_msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  

            self.vehicle_count+=1
            if self.vehicle_count > 200:
                self.vehicle_img_array.popleft()
            
            self.vehicle_img_array.append(img_bgr)
            self.is_vehicle_img = True

    def img_top_callback(self, img_msg):
        if not self.is_saving:
            np_arr = np.fromstring(img_msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  

            self.top_count+=1
            if self.top_count > 200:
                self.top_img_array.popleft()
            
            self.top_img_array.append(img_bgr)
            self.is_top_img = True

    def done_sig_callback(self,msg):
        self.is_done = True

if __name__ == '__main__':
    try:
        res_chkr = ResultChecker()
    except rospy.ROSInterruptException:
        pass