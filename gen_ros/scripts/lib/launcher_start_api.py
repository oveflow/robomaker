#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time,os

from lib.define import *
from lib.read_text import * 

from lib.controller import *

import subprocess


class launcher_start(controller):
        
    def __init__(self):                
        self.controller = controller()

    def launcher_start(self,map_list,scenario_list):

        
        while True:
            
            if self.controller.update():#simualtor data update
                self.controller.is_waitting() #check watting status
                self.controller.is_downloading() #check downloading status                                      

                if self.controller.is_befor_login():                                        
                    self.controller.commander(Command.LOGIN,user_id+'/'+user_pw)#Login명령
                    
                
                if self.controller.is_after_login() or self.controller.is_after_sim_quit_to_launcher():  # is_after_sim_quit_to_launcher : Simulator에서 quit 명령 후 Launcher 복귀 상태 확인

                    self.controller.commander(Command.SELECT_VER,version)#version 선택명령

                if self.controller.is_not_find_version(): #선택한 버전이 없는 버전인지 확인.
                    break
                    
                if self.controller.is_can_execute_sim():                                                                     
                    self.controller.commander(Command.EXECUTE_SIM,'') #Simulator 실행 명령                        

                                            
                if self.controller.is_sim_not_install():                                
                    self.controller.commander(Command.INSTALL_SIM,'') #Simulator 설치 명령                     

                if self.controller.is_sim_lobby():                                        
                    time.sleep(60)
                    break
                    
                else :
                    print("\033[A                                      \033[A")
                    print("[NO Simulator Control Data]")
                    time.sleep(1)            
                        

        for index, map in enumerate(map_list):        
            self.controller.commander(Command.MAP_VEHICLE_SELECT,map+'/'+vehicle)#시뮬레이션/옵션 변경 실행 명령
            print("MAP_SETTING_COMMAND_SEND")            
            time.sleep(10)
            while True:
                self.controller.update()
                if self.controller.is_sim_playing():                       
                    print("MAP_SETTING_DONE")

                    break
            #running alogorithim            
            task_id = os.environ['TASK_ID']              
            print(scenario_list,index)
            scenario_id = scenario_list[index]
            self.start_node_direct("gen_ros","gen_planner.launch",map,task_id,scenario_id)

        # terminated process   
        
        #tagging-simulation-job
        os.system('aws robomaker tag-resource --resource-arn $AWS_ROBOMAKER_SIMULATION_JOB_ARN --tags status=Completed')      

        # cancel-simulation-job
        os.system('aws robomaker cancel-simulation-job --job $AWS_ROBOMAKER_SIMULATION_JOB_ARN')
                        
    def start_node_direct(self,pkg,node,map_name,task_id,scenario_id):
        package = pkg
        node_name = node

        command = "roslaunch {0} {1} path:={2}".format(package,node_name,map_name)
        print(command)
        algorithm_p = subprocess.Popen(command,shell=True)


        # command = "rosrun {0} {1}".format('result_checker','main.py')
        
        result_command = 'rosrun result_checker main.py {0} {1} {2}'.format(task_id,scenario_id,map_name)    

        result_process = subprocess.Popen(result_command,shell=True)

        algorithm_p.wait()

        result_process.wait()

        # result_process.terminate()



