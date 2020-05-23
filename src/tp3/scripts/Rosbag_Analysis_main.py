import rosbag
import genpy
import Rosbag_Analysis as ra
import numpy as np
import matplotlib.pyplot as py

#BAGFILE = '../bagfiles/2020-05-13-15-10-05.bag'
BAGFILE = '../bagfiles/Groundtruth_2020-5-16-15-1-4.bag'
IoU_THRESHOLD = 0.8
TOPIC = 'objectlist'

# Main function for calling rosbag analysis methods
# --> is gonna be replaced by (or integrated) in GUI script 

array_objectIDs = ra.Rosbag_Analysis.getObjectIDs(BAGFILE)
print(array_objectIDs)

gt_object_count = ra.Rosbag_Analysis.getObjectCountTotal(BAGFILE)
print(gt_object_count)

bag = rosbag.Bag(BAGFILE)
#for topic, msg, t in bag.read_messages(topics=[TOPIC]):
    #print(msg.obj_list[0].classification.car)
    #attributes = genpy.message.get_printable_message_args(msg.obj_list[0].classification)
#print(attributes)

######################## get Raw Data ############################

# # generic implementation (in work) 
                                                                                                            # #bag file   #object id    #category      #attribute
#(array_timestamps_1, array_values_1) = ra.Rosbag_Analysis.getRawData(BAGFILE, 1, "", "obj_id")
# (array_timestamps_2, array_values_2) = ra.Rosbag_Analysis.getRawData(BAGFILE, 2, "geometric", "x")
# (array_timestamps_3, array_values_3) = ra.Rosbag_Analysis.getRawData(BAGFILE, 3, "geometric", "x")

# ### for showing the arrays
# print("Printing values out of generic function: ")
# for i in range(0, len(array_values_1), 1):
    # print(array_values_1[i])
# print(len(array_values_1))
# print(len(array_values_2))
# print(len(array_values_3))

# #plotting arrays
#py.plot(array_timestamps_1, array_values_1, '-b')
# py.plot(array_timestamps_2, array_values_2, '-r')
# py.plot(array_timestamps_3, array_values_3, '-k')
# py.show()     

######################## get Advanced Data ############################

#(timestamps, result) = ra.Rosbag_Analysis.getAdvancedData(BAGFILE, BAGFILE, 3, "geometric", "x", "difference", IoU_THRESHOLD)
#(timestamps, result) = ra.Rosbag_Analysis.getAdvancedData(BAGFILE, BAGFILE, 1, "", "obj_id", "difference")

(timestamps, TP) = ra.Rosbag_Analysis.getTP(BAGFILE, BAGFILE, IoU_THRESHOLD)
print(TP)

#plotting arrays
#py.plot(timestamps, result , '-b')
# py.plot(array_timestamps_2, array_values_2, '-r')
# py.plot(array_timestamps_3, array_values_3, '-k')
#py.show()  