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
import tf

OFFSET_CAR_X = -2.3 # distance to front
car_ego_x = 0
car_ego_y = 0
data_alt = 0
topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray,queue_size=10)
rospy.init_node('Objekt_Visualization')
br = tf.TransformBroadcaster()

#define each color to the specific class, input value ist the name(string) from the classifciation
def evaluateColor(Class): 
    class_List = {
	"car": [1,0,0,1],
	"truck":[0,1,0,1],
	"motorcycle": [0,0,1,1],
	"bicycle": [1,1,0,1],
	"pedestrian": [1,0,1,3],
	"stacionary": [0,1,1,3],
	"other":[1,1,1,2]   
    }
    return class_List.get(Class)
    
 
def evaluateClassification(objectClass):
    
    temp_prop = 0
    result = ""
    #tmp includes all Attributes of the message Classification
    tmp = [a for a in dir(objectClass) if not a.startswith('__') and not a.startswith('_') and not callable(getattr(objectClass,a))]
    

    for i in range(len(tmp)):
        if(getattr(objectClass, tmp[i]) > temp_prop ):
            temp_prop = getattr(objectClass, tmp[i])
            result = tmp[i]
    return (result) # return value is the name of the class whith the highest probability
            
    


def evaluateObject(objectData):
    marker = Marker()
    r, g, b, typ = evaluateColor(evaluateClassification(objectData.classification))
    marker.header.frame_id = "/base_link"
    
    marker.type = typ
    
    marker.action = marker.ADD
    marker.scale.x = objectData.dimension.length
    marker.scale.y = objectData.dimension.width
    
    marker.scale.z = objectData.dimension.height
    marker.color.a = 1.0
   
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = car_ego_x + objectData.geometric.x 
    marker.pose.position.y = car_ego_y + objectData.geometric.y * (-1)
    marker.pose.position.z = objectData.dimension.height/2
    marker.lifetime = rospy.Duration(0.1)
    return marker

def evaluateObjectID(objectData):
    marker = Marker()
    
    marker.header.frame_id = "/base_link"
    
    marker.type = marker.TEXT_VIEW_FACING
    
    marker.action = marker.ADD
    marker.scale.x = 2
    marker.scale.y = 2
    marker.scale.z = 1

    marker.color.a = 1.0
   
    marker.color.r = 0.3
    marker.color.g = 0.4
    marker.color.b = 1.0
    
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = car_ego_x + objectData.geometric.x 
    marker.pose.position.y = car_ego_y + objectData.geometric.y * (-1)
    marker.pose.position.z = objectData.dimension.height + 1
    marker.lifetime = rospy.Duration(0.1)
    marker.text = "ID:" + str(objectData.obj_id)
    return marker


def callback_simulation(data):

    global car_ego_x
    global car_ego_y 
    
    

    markerArray = MarkerArray()


    for i in range(len(data.obj_list)):
        markerObj = evaluateObject(data.obj_list[i])
        markerObj.id = i*2
        
        markerID = evaluateObjectID(data.obj_list[i])
        
        markerID.id = i*2+1
        markerArray.markers.append(markerObj)
        markerArray.markers.append(markerID)

    
    rospy.loginfo(markerArray)
    publisher.publish(markerArray)
    
   
def callback_egovehicle(data):
    global car_ego_x
    global car_ego_y

    car_ego_x = data.ego_geometric[0].x
    car_ego_y = data.ego_geometric[0].y
    
    


    br.sendTransform((OFFSET_CAR_X+car_ego_x,car_ego_y,0),
                     tf.transformations.quaternion_from_euler(0,0,1.57),
                     rospy.Time.now(),
                     "chassis",
                     "base_link")
    
    


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    

    #rospy.Subscriber("chatter", String, callback)
    rospy.Subscriber("simulation", ObjectsList, callback_simulation)
    rospy.Subscriber("egovehicle", ObjectsList, callback_egovehicle)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
