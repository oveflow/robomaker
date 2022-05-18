#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import subprocess

# def start_node_direct(pkg,node,arg):

#     package = pkg
#     node_name = node

#     command = "roslaunch {0} {1} path:={2}".format(package,node_name,arg)

#     p = subprocess.Popen(command,shell=True)

#     # p.terminate
#     # command = ['bash','rosrun result_checker main.py']
#     result_command = 'rosrun result_checker main.py {0} {1} {2}'.format()    
#     p_2 = subprocess.Popen(command)
    
#     p.wait()
    
#     print("AAAA")



# if __name__=="__main__":
#     rospy.init_node('test',anonymous=True)
#     start_node_direct("gen_ros","gen_planner.launch","R_KR_PR_SeongnamCityHall")

def map_scneario_lit():

    import os
    scenario_list=[]
    map_list=[]

    path = "/home/morai/Release/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario"

    sub_path = os.listdir("/home/morai/Release/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario")

    for sub in sub_path:
        for i in range(len(os.listdir(path+'/'+sub))):
            scenario_list.append(os.listdir(path+'/'+sub)[i].replace('.json',''))
            map_list.append(sub)

    print(scenario_list)

    print(map_list)
    # map_list = ['V_RHT_Suburb_02','R_KR_PG_KATRI','R_KR_PR_Sangam_NoBuildings','R_KR_PR_SeongnamCityHall','V_RHT_HighwayJunction_2']



def aws_cli_call_in_scripts__test():
    import os 
    # import subprocess

    # subprocess.call(['aws', 'robomaker', 'tag-resource', '--resource-arn' ,'$AWS_ROBOMAKER_SIMULATION_JOB_ARN', '--tags', 'status="complete"'])

    #cancel-simulation-job
    os.system('aws robomaker cancel-simulation-job --job $AWS_ROBOMAKER_SIMULATION_JOB_ARN')
    
    #tagging-simulation-job
    os.system('aws robomaker tag-resource --resource-arn $AWS_ROBOMAKER_SIMULATION_JOB_ARN --tags status='+status)

    status = 'complete'
    os.system('aws robomaker tag-resource --resource-arn $AWS_ROBOMAKER_SIMULATION_JOB_ARN --tags status='+status)


