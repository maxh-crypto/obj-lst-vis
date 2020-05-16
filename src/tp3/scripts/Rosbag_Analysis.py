import rosbag
import genpy.rostime
import numpy as np

# These functions can be called by the file "Rosbag_Analysis_main.py" #

TOPIC = '/simulation'
IoU_threshold = 0.0     # tbd

class Rosbag_Analysis:
  
    ### method for getting count of objects contained in rosbag ###
    @staticmethod
    def getObjectIDs(bagfile):
        bag = rosbag.Bag(bagfile)
        array_ids = []
        
        for topic, msg, t in bag.read_messages(topics=[TOPIC]):
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
    
    
    @staticmethod
    def getAdvancedData(bagfile1, bagfile2, obj_id_target_gt, category, attribute, operation):
    
        (timestamps_gt, values_gt, values_cam) = Rosbag_Analysis.getCalculationValues(bagfile1, bagfile2, obj_id_target_gt, category, attribute)
        
        ### operation ###
        if operation == "difference":
            array_result = np.subtract(values_gt, values_cam)
        
        return (timestamps_gt, array_result)
     ######################  


     
    @staticmethod
    #returns (timestamp_gt, values_gt, values_cam) 
    def getCalculationValues(bagfile1, bagfile2, obj_id_target_gt, category, attribute):
        
        bag_gt = rosbag.Bag(bagfile1)
        bag_cam = rosbag.Bag(bagfile2)
		       
        startTimeRaw_gt= bag_gt.get_start_time()
        startTime_gt = genpy.rostime.Time.from_sec(startTimeRaw_gt)
        
        startTimeRaw_cam= bag_cam.get_start_time()
        startTime_cam = genpy.rostime.Time.from_sec(startTimeRaw_cam)

        
        array_msg_cam = []
        
        array_values_gt = []
        array_timestamps_gt = []
        array_values_cam = []

        # loop through GT messages
        for topic, msg, t in bag_gt.read_messages(topics=[TOPIC]):
            bag_gt.read_messages()
            timestamp_gt = (float)((t.__sub__(startTime_gt)).__str__()) / 1000000   #normed timestamp GT in ms
            array_timestamps_gt.append(timestamp_gt)
            
            obj_gt = 0
            index_counter = 0
            obj_id_index = 99
            
            obj_cam = 0
            value_cam_found = 0
            
            # ID mapping:
            # loop through GT objects in message --> get right GT object from ID 
            for i in msg.obj_list:
                if i.obj_id == obj_id_target_gt:      
                                # timestamp       # GT object    
                    obj_gt = [timestamp_gt, msg.obj_list[index_counter]]
                    array_values_gt.append(getattr(getattr(obj_gt[1], category), attribute))
                    break
                else: index_counter += 1 # next object
           
            # finding concerning CAM object 
            # loop through CAM messages
            for topic, msg, t in bag_cam.read_messages(topics=[TOPIC]):
                bag_cam.read_messages()
                timestamp_cam = (float)((t.__sub__(startTime_cam)).__str__()) / 1000000   #normed timestamp CAM in ms
                
                # time mapping: 
                # cam message in range +/- 500 ms from gt message
                if((timestamp_cam > (timestamp_gt - 500))
                and (timestamp_cam < (timestamp_gt + 500))):
                    
                    # position mapping: 
                    # loop through CAM objects in message --> compare all CAM objects with GT object
                    if value_cam_found == 0:
                        for i in msg.obj_list:
                            prop_IoU = 0
                                            #index 0: timest.  #index 1: object
                            obj_cam = [timestamp_cam, i]
                          
                            ###
                            #   IoU testing: each CAM object compared to current GT object
                            #prop_IoU = testIoU(obj_gt[1], obj_cam[1])
                            ###
                            
                            if prop_IoU >= IoU_threshold:
                                last_value_cam = getattr(getattr(obj_cam[1], category), attribute)
                                array_values_cam.append(getattr(getattr(obj_cam[1], category), attribute))
                                value_cam_found = 1
                                break
                        
            if value_cam_found == 0:
                array_values_cam.append(last_value_cam)
                print('insert last value: ')
                print(last_val_cam)
        
        bag_gt.close()
        bag_cam.close()
        #print(array_timestamps_gt)
        #print("****")
        #print(array_values_gt)
        #print("****")
        #print(array_values_cam)
        return (array_timestamps_gt, array_values_gt, array_values_cam)
    ######################        
     
     
    
    ### example method - DEPRECATED ###
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
