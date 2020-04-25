import rosbag

# These functions can be called by the file "Rosbag_Analysis_main.py" #

class Rosbag_Analysis:
    
    
    ### example method ###
    ### get all x values of object 1 (index 0) ###
    
    @staticmethod
    def get_geometric_x(bagfile, obj_id):
        
        bag = rosbag.Bag(bagfile)
        
        counter = 0
        array_x = []

        for topic, msg, t in bag.read_messages(topics=['camera_obj']):
            counter += 1
            bag.read_messages()
          
            array_x.append(msg.obj_list[obj_id].geometric.x)

        #for i in range(0, len(array_x), 1):
            #print(array_x[i])
         
        #print('Number of sequences: ')     #sequence = count of frames
        #print(counter)
        
        bag.close()
        return array_x
    ######################   
     

    #print(msg)
