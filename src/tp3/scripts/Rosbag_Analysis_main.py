import Rosbag_Analysis as ra
import numpy as np
import matplotlib.pyplot as py

BAGFILE = '../bagfiles/2020-05-13-15-10-05.bag'

# Main function for calling rosbag analysis methods
# --> is gonna be replaced by (or integrated) in GUI script 

array_objectIDs = ra.Rosbag_Analysis.getObjectIDs(BAGFILE)
print(array_objectIDs)

# generic implementation (in work) 
                                                                                                            #bag file   #object id    #category      #attribute
(array_timestamps_1, array_values_1) = ra.Rosbag_Analysis.getRawData(BAGFILE, 1, "geometric", "x")
(array_timestamps_2, array_values_2) = ra.Rosbag_Analysis.getRawData(BAGFILE, 2, "geometric", "x")
(array_timestamps_3, array_values_3) = ra.Rosbag_Analysis.getRawData(BAGFILE, 3, "geometric", "x")

### for showing the arrays
print("Printing values out of generic function: ")
for i in range(0, len(array_values_1), 1):
    print(array_values_1[i])
print(len(array_values_1))
print(len(array_values_2))
print(len(array_values_3))

#plotting arrays
py.plot(array_timestamps_1, array_values_1, '-b')
py.plot(array_timestamps_2, array_values_2, '-r')
py.plot(array_timestamps_3, array_values_3, '-k')
py.show()     



#prototype
#(array_timestamps, array_values_difference) = ra.Rosbag_Analysis.getAdvancedData('../bagfiles/Objektliste.bag', '../bagfiles/Objektliste.bag', 0, 1, "dimension", "width", "difference")



   
#print("Printing  values out of generic function: ")
#for i in range(0, len(array_values_y), 1):
#    print(array_values_y[i])

#print("Printing  values out of advanced function - difference: ")
#for i in range(0, len(array_values_difference), 1):
#    print(array_values_difference[i])
  
# for calculating differnces between arrays:
  
#diff = []
#diff = np.subtract(x_array_obj1, x_array_obj2)
#print(diff)
