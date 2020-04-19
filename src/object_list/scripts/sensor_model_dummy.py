#!/usr/bin/env python
# license removed for brevityi

import rospy #permit to use python 
import numpy as np
from object_list.msg import ObjectsList
from object_list.msg import ObjectList
import math


def sensor_model_dummy():

   pub = rospy.Publisher('camera_obj', ObjectsList, queue_size=100) #
   rospy.init_node('camera',anonymous=False)  # Initiate the node camera and anonymous true permitt openinig this node a lot of time including number in the end of the node name  
   rate=rospy.Rate(50)  #1 hz

   while not rospy.is_shutdown():

        b=ObjectsList()
        a1=ObjectList()
	a1.obj_id= 1
        a1.geometric.x = 5 * math.sin(rospy.get_time() )
        a1.geometric.y = 5 * math.cos(rospy.get_time())
	a1.dimension.length = rospy.get_time() %5
	a1.dimension.width= 1    
        a1.classification.car = 0.9    

        a2=ObjectList()
	a2.obj_id= 2
        a2.geometric.x = 2 * math.cos(rospy.get_time() )
        a2.geometric.y = 2 * math.sin(rospy.get_time())
	a2.dimension.length = rospy.get_time() %6
	a2.dimension.width= rospy.get_time() % 3  
        a2.classification.pedestrian = 0.5  
        
        b.header.stamp = rospy.Time.now()
        b.header.frame_id = "sensor_model_dummy"
        b.obj_list = np.append(a1,a2)
	#rospy.loginfo(b)
        rospy.loginfo(a1.geometric)
        rospy.loginfo(a1.classification)
        rospy.loginfo(a2.geometric)
        rospy.loginfo(a2.classification)
        pub.publish(b)
        rate.sleep()
	

if __name__ == '__main__':
    try:
        sensor_model_dummy()
    except rospy.ROSInterruptException:
        pass
