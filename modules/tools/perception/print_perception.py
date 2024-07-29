#!/usr/bin/env python3

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################


#!/usr/bin/env python3

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time

import math
import time
import os
import numpy as np
from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import PerceptionObstacles
from modules.common_msgs.planning_msgs.planning_pb2 import ADCTrajectory,RSSInfo


class ApolloNode:

    def __init__(self):
        cyber.init()
        self.node = cyber.Node("obstacles_publisher")
        self.obstacle_writer = self.node.create_writer('/apollo/perception/obstacles', PerceptionObstacles)
        self.traj_writer = self.node.create_writer('/apollo/planning', ADCTrajectory)
 

if __name__ == '__main__':
    apollo_node = ApolloNode()
    seq=0
    print('publishing trajectory ...')

    map= "apollo_virtual"
    theta = 1.57#1.35 # radian
    theta2 = 1.57#1.35 # radian
    
    T = 0.09   #in seconds/ sleep time

    start_time = cyber_time.Time.now().to_sec()
    if map == "apollo_virtual":
        start_point = [751007.88 ,2563971.37] #[750965.33 ,2563924.31]
    curr_x = start_point[0]
    curr_y = start_point[1]
    curr_v = 0.1
    curr_theta = 0#-1.6

    k = 1
    while not cyber.is_shutdown():

        
        t = np.arange(0.1, 7.2, 0.1) # start, end , step
        t = t.tolist()
        full_a = [0.3]* len(t)
        full_v = [(curr_v + full_a[0]* ti) for ti in t]
        full_x = [(curr_x + (math.cos(curr_theta)*curr_v)* ti + math.cos(curr_theta)*full_a[0]* ti* ti ) for ti in t]
        full_y = [(curr_y + (math.sin(curr_theta)*curr_v)* ti + math.sin(curr_theta)*full_a[0]* ti* ti ) for ti in t]
        full_heading = [curr_theta]*len(t)
        
        
        full_relative_time = list(range(0,700,7))
        full_relative_time = [float(x)/100.0 for x in full_relative_time]
        print("relative t ",full_relative_time)
        msg = ADCTrajectory()
        msg.header.module_name = 'planning'
        msg.header.sequence_num = seq
        msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        msg.header.lidar_timestamp = 0
        msg.header.radar_timestamp = 0
        msg.header.camera_timestamp = 0
        
        for i in range(len(full_x)):
            traj_p = msg.trajectory_point.add()
            traj_p.v = full_v[i]
            traj_p.a = full_a[i]
            traj_p.relative_time = full_relative_time[i] -full_relative_time[k]
            print(i,"rel time ", full_relative_time[i] -full_relative_time[k])
            
            path_p = traj_p.path_point
            path_p.x = full_x[i]
            path_p.y = full_y[i]
            path_p.z = 0
            path_p.theta = full_heading[i]
            path_p.kappa = 0.0
            path_p.dkappa = 0.0
            path_p.ddkappa = 0.0
            
            path_p.s = full_x[k]- full_x[i]
            print(i,"s ", full_x[k]- full_x[i])
        
        msg.trajectory_type = 1
        msg.total_path_time = full_relative_time[-1]-full_relative_time[0]
        msg.total_path_length = full_x[0] - full_x[-1]

        msg.is_replan = 0
        msg.gear = 1

        msg.rss_info.is_rss_safe = 1

        msg.rss_info.cur_dist_lon= 3000.000000000
        msg.rss_info.rss_safe_dist_lon= 3000.000000000
        msg.rss_info.acc_lon_range_minimum= -8.000000000
        msg.rss_info.acc_lon_range_maximum= 3.500000000
        msg.rss_info.acc_lat_left_range_minimum= -0.800000000
        msg.rss_info.acc_lat_left_range_maximum= 0.200000000
        msg.rss_info.acc_lat_right_range_minimum= -0.800000000
        msg.rss_info.acc_lat_right_range_maximum= 0.200000000
        
        k+=1
        if (k==20):
            k = 1

        seq= seq+1
        apollo_node.traj_writer.write(msg) 
        print("trajectory was published...")
        curr_x = full_x[1]
        curr_y = full_y[1]
        curr_v = full_v[1]
        time.sleep(T)


    """
    	###### static obstacle #################
        msg = PerceptionObstacles()
        msg.header.module_name = 'perception_obstacle'
        msg.header.sequence_num = seq
        msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        msg.header.lidar_timestamp = cyber_time.Time.now().to_nsec()
        
        obstacle = msg.perception_obstacle.add()
        
        obstacle.id = 1
        obstacle.theta = theta   # in radian
        obstacle.position.x = 57.34 #587196.86
        obstacle.position.y = -142.00 # 4141414.31
        obstacle.position.z = 0
    
        
        obstacle.velocity.x = 0
        obstacle.velocity.y = 0
        obstacle.velocity.z = 0

        obstacle.length = 4.565
        obstacle.width = 2.082
        obstacle.height = 1.35
        

        obstacle.tracking_time = cyber_time.Time.now().to_sec() - start_time
        
        obstacle.type = 2
        obstacle.timestamp = time.time()
        
        ##########################################
        
        
        ########   obstacle ##################
        obstacle2 = msg.perception_obstacle.add()
        obstacle2.id =2
        obstacle2.theta = theta2   # in radian
        obstacle2.position.x = 66.74#587202.08
        obstacle2.position.y = -142.00#4141413.09
        obstacle2.position.z = 0
        
        
        obstacle2.velocity.x = 0
        obstacle2.velocity.y = 0
        obstacle2.velocity.z = 0
        obstacle2.length = 4.565
        obstacle2.width = 2.082
        obstacle2.height = 1.35
        obstacle2.tracking_time = cyber_time.Time.now().to_sec() - start_time
        obstacle2.type = 5
        obstacle2.timestamp = time.time()
        ### static obstacle




        ###########################################
        seq= seq+1
        apollo_node.obstacle_writer.write(msg)  
        time.sleep(T)      
    """
   
   
   
   
   
   
   
   


# """
# print received perception message
# """
# import argparse
# from cyber.python.cyber_py3 import cyber

# from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import PerceptionObstacles


# def receiver(data):
#     """receiver"""
#     print(data)


# def perception_receiver(perception_channel):
#     """publisher"""
#     cyber.init()
#     node = cyber.Node("perception")
#     node.create_reader(perception_channel, PerceptionObstacles, receiver)
#     print("test")
#     node.spin()


# if __name__ == '__main__':
#     parser = argparse.ArgumentParser(description="create fake perception obstacles",
#                                      prog="print_perception.py")
#     parser.add_argument("-c", "--channel", action="store", type=str,
#                         default="/apollo/perception/obstacles",
#                         help="set the perception channel")

#     args = parser.parse_args()
#     perception_receiver(args.channel)
