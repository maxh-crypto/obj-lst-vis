#!/usr/bin/env python
import roslib; roslib.load_manifest('visualization_marker_tutorials')
import rospy
from std_msgs.msg import String
from object_list.msg import ObjectsList
from object_list.msg import ObjectList

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray,queue_size=10)

rospy.init_node('register')

markerArray = MarkerArray()


def callback(data):
   rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.obj_list[0].obj_id)
   marker = Marker()
   marker.header.frame_id = "/neck"
   marker.type = marker.CUBE
   marker.action = marker.ADD
   marker.scale.x = data.obj_list[0].dimension.length
   marker.scale.y = data.obj_list[0].dimension.width
   marker.scale.z = 1
   marker.color.a = 1.0
   marker.color.r = 1.0
   marker.color.g = 1.0
   marker.color.b = 0.0
   marker.pose.orientation.w = 1.0
   marker.pose.position.x = data.obj_list[0].geometric.x 
   marker.pose.position.y = data.obj_list[0].geometric.y
   marker.pose.position.z = 0
   marker.id =0
   markerArray.markers.append(marker)

   publisher.publish(markerArray)
   
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    

    #rospy.Subscriber("chatter", String, callback)
    rospy.Subscriber("camera_obj", ObjectsList, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
