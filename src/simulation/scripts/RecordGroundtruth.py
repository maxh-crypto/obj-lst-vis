#!/usr/bin/env python
import roslib; roslib.load_manifest('visualization_marker_tutorials')
import rospy
from std_msgs.msg import String
from object_list.msg import ObjectsList
from object_list.msg import ObjectList

import rospy
import rosnode
import math
import rosbag
import datetime
import os
import rospkg

rospack = rospkg.RosPack()
rospack.list()


rospy.init_node('Record_Objektlist_Groundtruth')
root_path =  rospack.get_path('simulation') + '/rosbag_files/' 
current_time = datetime.datetime.now()
path  = root_path + 'Groundtruth_' + str(current_time.year) + '-' + str(current_time.month) + '-' + str(current_time.day) + '-' + str(current_time.hour) + '-' + str(current_time.minute) + '-' + str(current_time.second) + '.bag'


bag = rosbag.Bag(path,'w')

def callback_simulation(data):
    
    bag.write('objectlist',data)
   
def callback_egovehicle(data):
    
    bag.write('egovehicle',data)
def listener():

    rospy.Subscriber("simulation", ObjectsList, callback_simulation)
    rospy.Subscriber("egovehicle", ObjectsList, callback_egovehicle)
    # spin() simply keeps python from exiting until this node is stopped
    
    
    rospy.spin()
    bag.close()

if __name__ == '__main__':
    listener()
