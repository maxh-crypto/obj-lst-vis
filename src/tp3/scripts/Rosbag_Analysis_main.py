import Rosbag_Analysis as ra
import numpy as np

# Main function for calling rosbag analysis methods
# --> is gonna be replaced by (or integrated) in GUI script 

x_array_obj1 = []   # array of all x values of object 1
x_array_obj2 = []   # array of all x values of object 2

x_array_obj1 = ra.Rosbag_Analsysis.get_geometric_x('../bagfiles/2020-04-24-18-36-14.bag', 1)
x_array_obj2 = ra.Rosbag_Analysis.get_geometric_x('../bagfiles/2020-04-24-18-36-14.bag', 2)

# for showing the arrays
for i in range(0, len(x_array_obj1), 1):
    print(x_array_obj1[i])

for i in range(0, len(x_array_obj2), 1):
    print(x_array_obj2[i])
  
# just as example (makes no sense in the project):
# calculate the difference between each array values (x1_obj1 - x1_obj2)
# and print difference array
  
diff = []
diff = np.subtract(x_array_obj1, x_array_obj2)
print(diff)