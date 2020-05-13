import rosbag
import genpy.rostime
import numpy as np

# These functions can be called by the file "Rosbag_Analysis_main.py" #

TOPIC = '/simulation'

class Rosbag_Analysis:
  
    ### method for getting count of objects contained in rosbag ###
    @staticmethod
    def getObjectIDs(bagfile):
        bag = rosbag.Bag(bagfile)
        array_ids = []
        
        for topic, msg, t in bag.read_messages(topics=[TOPIC]):
        #for topic, msg, t in bag.read_messages(topics=['objectlist']) and len(msg.obj_list) > 0:
            for i in msg.obj_list:
                if i.obj_id not in array_ids:
                    array_ids.append(i.obj_id)
               
        np.sort(array_ids)
        bag.close()
        return array_ids
    
    
    ### method for getting array of raw values of all frames in a rosbag ###
    ### generic implementation ###
    
    @staticmethod
    def getRawData(bagfile, obj_id_target, category, attribute):
        
        bag = rosbag.Bag(bagfile)
        
        startTimeRaw= bag.get_start_time()
        startTime = genpy.rostime.Time.from_sec(startTimeRaw)

        array_values = []
        array_timestamps = []

        # loop through all messages in bagfile
        for topic, msg, t in bag.read_messages(topics=[TOPIC]):
            bag.read_messages()
            index_counter = 0
            obj_id_index = 99
            
            # loop through all objects in message
            # to find array index of target object ID
            for i in msg.obj_list:
                if i.obj_id == obj_id_target:
                    array_values.append(getattr(getattr(msg.obj_list[index_counter], category), attribute))
                    array_timestamps.append((float)((t.__sub__(startTime)).__str__()) / 1000000)  #timestamp in milli seconds
                    break
                else: index_counter += 1
                    
            #if obj_id_index == 99:  # id not found in message # insert value '0' if object ID can be found in message
                #array_values.append(0)
                #array_timestamps.append((float)((t.__sub__(startTime)).__str__()) / 1000000)  #appends timestamp in milli seconds
                #break
        
        bag.close()
        return (array_timestamps, array_values)
    ######################   
    
    # prototype
    ### method for getting array of calculated values of all frames in a rosbag ###
    ### generic implementation ###
    
    @staticmethod
    def getAdvancedData(bagfile1, bagfile2, obj_id1, obj_id2, category, attribute, operation):
        
        bag1 = rosbag.Bag(bagfile1)
        bag2 = rosbag.Bag(bagfile2)
		       
        startTimeRaw1= bag1.get_start_time()
        startTime1 = genpy.rostime.Time.from_sec(startTimeRaw1)
        
        startTimeRaw2= bag2.get_start_time()
        startTime2 = genpy.rostime.Time.from_sec(startTimeRaw2)

        counter1 = 0
        counter2 = 0
        array_values1 = []
        array_values2 = []
        array_result = []
        array_timestamps1 = []
        array_timestamps2 = []

        for topic, msg, t in bag1.read_messages(topics=[TOPIC]):
            counter1 += 1
            bag1.read_messages()
            
            array_values1.append(getattr(getattr(msg.obj_list[obj_id1], category), attribute))
            
            array_timestamps1.append((float)((t.__sub__(startTime1)).__str__()) / 1000000)  #appends timestamp in milli seconds
           
        #print('Number of sequences: ')     #sequence = count of frames
        #print(counter)
        
        for topic, msg, t in bag2.read_messages(topics=[TOPIC]):
            counter2 += 1
            bag2.read_messages()
            
            array_values2.append(getattr(getattr(msg.obj_list[obj_id2], category), attribute))
            
            array_timestamps2.append((float)((t.__sub__(startTime2)).__str__()) / 1000000)  #appends timestamp in milli seconds
           
        #print('Number of sequences: ')     #sequence = count of frames
        #print(counter)
        
        ### operation ###
        if(operation == "difference"):
            array_result = np.subtract(array_values1, array_values2)
        
        ### etc...
        
        
        bag1.close()
        bag2.close()
        return (array_timestamps1, array_result) 
        # welche Timestamps sollen zurueckgegeben werden???
        ###################### 
     
    
    ### example method ###
    ### get all x values of object 1 ( = index 1) ###
    ### following method is deprecated ###
    
    @staticmethod
    def get_geometric_x(bagfile, obj_id): 
        
        bag = rosbag.Bag(bagfile)
              
        startTimeRaw= bag.get_start_time()
        startTime = genpy.rostime.Time.from_sec(startTimeRaw)

        counter = 0
        array_x = []
        array_timestamps = []
        
        for topic, msg, t in bag.read_messages(topics=[TOPIC]):
            counter += 1
            bag.read_messages()
            
            array_x.append(msg.obj_list[obj_id].geometric.x)
            array_timestamps.append((float)((t.__sub__(startTime)).__str__()) / 1000000)  #appends timestamp in milli seconds
           
        #print('Number of sequences: ')     #sequence = count of frames
        #print(counter)
        
        bag.close()
        return (array_timestamps, array_x)
    ######################   
