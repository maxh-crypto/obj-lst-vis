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

rospy.init_node('Record_Objektlist')
root_path = os.environ['HOME'] + '/obj-lst-vis/src/tp3/bagfiles/' 
current_time = rospy.Time
path  = root_path + 'Objektliste_' + str(datetime.datetime.now()) + '.bag'
rospy.loginfo (datetime.datetime.now())

bag = rosbag.Bag(path,'w')

def callback(data):
    
    bag.write('camera_obj',data)
   
def listener():

    rospy.Subscriber("camera_obj", ObjectsList, callback)
    # spin() simply keeps python from exiting until this node is stopped
    
    
    rospy.spin()
    bag.close()

if __name__ == '__main__':
    listener()
