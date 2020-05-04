import Rosbag_Analysis as ra
import numpy as np

# Main function for calling rosbag analysis methods
# --> is gonna be replaced by (or integrated) in GUI script 

x_array_obj1 = []   # array of all x values of object 1

# example: getting geometric.x values for simple plot
(timestamps1, x_array_obj1) = ra.Rosbag_Analysis.get_geometric_x('../../bagfiles/Objektliste_2020-04-26-12-47-33.bag', 1)

# generic implementation (in work) 
                                                                                                                                 #bag file                               #object id     #category      #attribute
(array_timestamps, array_values_x) = ra.Rosbag_Analysis.getRawData('../../bagfiles/Objektliste_2020-04-26-12-47-33.bag', 1, "geometric", "x")
(array_timestamps, array_values_y) = ra.Rosbag_Analysis.getRawData('../../bagfiles/Objektliste_2020-04-26-12-47-33.bag', 1, "geometric", "y")

### for showing the arrays

print("Printing  timestamps in ms out of generic function: ")
for i in range(0, len(array_timestamps), 1):
    print(array_timestamps[i])

#print("Printingx values in ms out of simple function (not generic): ")
#for i in range(0, len(x_array_obj1), 1):
#    print(x_array_obj1[i])

print("Printing  X values out of generic function: ")
for i in range(0, len(array_values_x), 1):
    print(array_values_x[i])
    
print("Printing  Y values out of generic function: ")
for i in range(0, len(array_values_y), 1):
    print(array_values_y[i])


  
# for calculating differnces between arrays:
  
#diff = []
#diff = np.subtract(x_array_obj1, x_array_obj2)
#print(diff)
