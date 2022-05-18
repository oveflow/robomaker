#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys,os, signal
from lib.controller import *
from lib.launcher_start_api import *
from lib.read_text import *
"""
https://docs.google.com/spreadsheets/d/1jHbR_JoZFYfxMirwSp-48peWkJf1xUmMZyFIwetxcZM/edit#gid=0

"""

class api :

    def __init__(self):         

        signal.signal(signal.SIGINT, self.signal_handler) #handle ctrl-c
        map_list=[]
        scenario_list=[]

        path = "/home/morai/Release/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario"

        sub_path = os.listdir("/home/morai/Release/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario")

        for sub in sub_path:
            for i in range(len(os.listdir(path+'/'+sub))):
                scenario_list.append(os.listdir(path+'/'+sub)[i].replace('.json',''))
                map_list.append(sub)
        
        map_list=['R_KR_PG_KATRI','R_KR_PR_Sangam_NoBuildings','R_KR_PR_SejongBRT0','V_RHT_HighwayJunction_2','V_RHT_Suburb_02']        

        
        api = launcher_start()
        api.launcher_start(map_list,scenario_list)   

    def signal_handler(self, signal, frame):        
        sys.exit(0)                                           

if __name__ == "__main__":
    start=api()
    
