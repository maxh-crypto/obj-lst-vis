import Rosbag_Analysis as ra
import numpy as np

# Main function for calling rosbag analysis methods
# --> is gonna be replaced by (or integrated) in GUI script 

object_count = ra.Rosbag_Analysis.getObjectCount('../bagfiles/Objektliste.bag')
print(object_count)

# generic implementation (in work) 
                                                                                                        #bag file                #object id    #category      #attribute
(array_timestamps, array_values_x) = ra.Rosbag_Analysis.getRawData('../bagfiles/Objektliste.bag', 0, "dimension", "length")
(array_timestamps, array_values_y) = ra.Rosbag_Analysis.getRawData('../bagfiles/Objektliste.bag', 0, "classification", "car")

#deprecated:
# example: getting geometric.x values for simple plot
x_array_obj1 = []   # array of all x values of object 1
(timestamps1, x_array_obj1) = ra.Rosbag_Analysis.get_geometric_x('../bagfiles/Objektliste.bag', 1)


### for showing the arrays

#print("Printing  timestamps in ms out of generic function: ")
#for i in range(0, len(array_timestamps), 1):
#    print(array_timestamps[i])

#print("Printing x values in ms out of simple function (not generic): ")
#for i in range(0, len(x_array_obj1), 1):
#    print(x_array_obj1[i])

print("Printing values out of generic function: ")
for i in range(0, len(array_values_x), 1):
    print(array_values_x[i])
    
print("Printing  values out of generic function: ")
for i in range(0, len(array_values_y), 1):
    print(array_values_y[i])


  
# for calculating differnces between arrays:
  
#diff = []
#diff = np.subtract(x_array_obj1, x_array_obj2)
#print(diff)
