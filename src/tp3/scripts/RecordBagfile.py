#!/usr/bin/env python
import roslib; roslib.load_manifest('visualization_marker_tutorials')
import rospy
from std_msgs.msg import String
from object_list.msg import ObjectsList
from object_list.msg import ObjectList

import rospy
import math

topic = 'visualization_marker_array'

rospy.init_node('Record_Objektlist')
  
 
d

def callback(data):
   
    markerArray = MarkerArray()
    rospy.loginfo(data.obj_list[0].geometric)
    for i in range(len(data.obj_list)):
        markerObj = evaluateObject(data.obj_list[i])
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.obj_list[i])
        markerObj.id = i
        markerArray.markers.append(markerObj)

    publisher.publish(markerArray)
   
def listener():

    rospy.Subscriber("camera_obj", ObjectsList, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
