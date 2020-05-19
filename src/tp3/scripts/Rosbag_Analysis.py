import rosbag
import genpy.rostime
import numpy as np
import sys
sys.path.append('../src/tp3')
import DataEvaluation_module as de

# These functions can be called by the file "Rosbag_Analysis_main.py" #

TOPIC = 'objectlist'

class Rosbag_Analysis:
  
    ### method for getting available object IDs contained in rosbag ###
    @staticmethod
    def getObjectIDs(bagfile):
        bag = rosbag.Bag(bagfile)
        array_ids = []
        
        for topic, msg, t in bag.read_messages(topics=[TOPIC]):
            for i in msg.obj_list:
                if i.obj_id not in array_ids:
                    array_ids.append(i.obj_id)
               
        return_array = np.sort(array_ids)
        bag.close()
        return return_array
        
    ### method for getting count of objects contained in rosbag summed up over all messages ###
    @staticmethod
    def getObjectCountTotal(bagfile):
        bag = rosbag.Bag(bagfile)
        objects_msg = []
        count_objects_total = 0
       
        for topic, msg, t in bag.read_messages(topics=[TOPIC]):
            for i in msg.obj_list:
                objects_msg.append(i)
                count_objects_total += len(objects_msg)
               
        bag.close()
        return count_objects_total
    
    
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
                    if category == "": # branch with one level in object list tree (e.g. 'obj_id')
                        array_values.append(getattr(msg.obj_list[index_counter], attribute))
                    else:
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
    def getAdvancedData(bagfile1, bagfile2, obj_id_target_gt, category, attribute, operation, IoU_threshold):
    
        (timestamps_gt, values_gt, values_cam) = Rosbag_Analysis.getCalculationValues(bagfile1, bagfile2, obj_id_target_gt, category, attribute, IoU_threshold)
        
        ### operation ###
        if operation == "difference":
            array_result = np.subtract(values_gt, values_cam)
            
        return (timestamps_gt, array_result)
     ######################  
     
     
     
    @staticmethod
    #returns (timestamp_gt, values_gt, values_cam) 
    def getCalculationValues(bagfile1, bagfile2, obj_id_target_gt, category, attribute, IoU_threshold):
        
        bag_gt = rosbag.Bag(bagfile1)
        bag_cam = rosbag.Bag(bagfile2)
		       
        startTimeRaw_gt= bag_gt.get_start_time()
        startTime_gt = genpy.rostime.Time.from_sec(startTimeRaw_gt)
        
        startTimeRaw_cam= bag_cam.get_start_time()
        startTime_cam = genpy.rostime.Time.from_sec(startTimeRaw_cam)
        
        array_timestamps_gt = []
        array_values_gt = []
        array_values_cam = []
        
        # loop through GT messages/frames
        for topic, msg, t in bag_gt.read_messages(topics=[TOPIC]):
            bag_gt.read_messages()
            timestamp_gt = (float)((t.__sub__(startTime_gt)).__str__()) / 1000000   #normed timestamp GT in ms
            array_timestamps_gt.append(timestamp_gt)
            
            obj_gt = 0
            index_counter = 0
            obj_id_index = 99
            
            array_objects_gt = []
            
            obj_cam = 0
            value_cam_found = 0
            last_value_cam = 0
            
            # ID mapping:
            # loop through GT objects in message --> get right GT object from ID 
            for i in msg.obj_list:
                #collect all GT objects
                array_objects_gt.append(i)
                
                #get calculation values of target GT object with timestamp
                if i.obj_id == obj_id_target_gt:      
                                # timestamp       # GT object    
                    obj_gt = [timestamp_gt, msg.obj_list[index_counter]]
                    if category == "": # branch with one level in object list tree (e.g. 'obj_id')
                        array_values_gt.append(getattr(obj_gt[1], attribute))
                    else:
                        array_values_gt.append(getattr(getattr(obj_gt[1], category), attribute))
                else: index_counter += 1 # next object
      
            # finding concerning CAM object 
            # loop through CAM messages/frames to find message concerning to GT frame (in time) 
            for topic, msg, t in bag_cam.read_messages(topics=[TOPIC]):
                bag_cam.read_messages()
                timestamp_cam = (float)((t.__sub__(startTime_cam)).__str__()) / 1000000   #normed timestamp CAM in ms
                
                # time mapping: 
                # cam message in range +/- 500 ms from gt message
                if((timestamp_cam > (timestamp_gt - 500))
                and (timestamp_cam < (timestamp_gt + 500))):
                    
                    # position mapping: 
                    # loop through CAM objects in message --> proceed IoU evaluation
                    if value_cam_found == 0: # for time mapping # PROBLEMATISCH???
                        for i in msg.obj_list:
                                            #index 0: timest.  #index 1: object
                            obj_cam = [timestamp_cam, i]
                          
                            ###
                            #IoU testing: each CAM object compared to list of GT objects
                            (evaluation, index_TP) = de.det_TP_FP_mm(array_objects_gt, obj_cam[1], IoU_threshold)
                            ###

                            # TP (true positive) case - you got a match:
                            if evaluation == 0 and array_objects_gt[index_TP].obj_id == obj_id_target_gt:
                                if category == "": # branch with one level in object list tree (e.g. 'obj_id')
                                    last_value_cam = getattr(obj_cam[1], attribute)
                                    array_values_cam.append(getattr(obj_cam[1], attribute))
                                else:
                                    last_value_cam = getattr(getattr(obj_cam[1], category), attribute)
                                    array_values_cam.append(getattr(getattr(obj_cam[1], category), attribute))
                                value_cam_found = 1
                                break

            if value_cam_found == 0:
                array_values_cam.append(last_value_cam)
                #print('insert last value: ')
                #print(last_val_cam)
        
        bag_gt.close()
        bag_cam.close()
        #print(array_timestamps_gt)
        #print("****")
        #print(array_values_gt)
        #print("****")
        #print(array_values_cam)
        return (array_timestamps_gt, array_values_gt, array_values_cam)
    ######################        
     
    @staticmethod
    def getEvaluation(bagfile1, bagfile2, IoU_threshold):
        
        bag_gt = rosbag.Bag(bagfile1)
        bag_cam = rosbag.Bag(bagfile2)
		       
        startTimeRaw_gt= bag_gt.get_start_time()
        startTime_gt = genpy.rostime.Time.from_sec(startTimeRaw_gt)
        
        startTimeRaw_cam= bag_cam.get_start_time()
        startTime_cam = genpy.rostime.Time.from_sec(startTimeRaw_cam)
        
        array_timestamps_gt = []
        FP_total = 0
        count_GT_objects_total = 0
        
        # loop through GT messages/frames
        for topic, msg, t in bag_gt.read_messages(topics=[TOPIC]):
            bag_gt.read_messages()
            timestamp_gt = (float)((t.__sub__(startTime_gt)).__str__()) / 1000000   #normed timestamp GT in ms
            array_timestamps_gt.append(timestamp_gt)
            
            obj_gt = 0
            index_counter = 0
            obj_id_index = 99
            
            array_objects_gt = []
            array_objects_cam = []
            
            obj_cam = 0
            #value_cam_found = 0
            
            count_TP = 0
            count_FP = 0 
            count_mm = 0
            count_FN = 0
           
            array_precision = []
            array_recall = []
            array_TP = []
            array_FP = []
            array_FN = []
            array_mm = []
            
            '''           
            # ID mapping:
            # loop through GT objects in message --> get right GT object from ID 
            for i in msg.obj_list:
                #collect all GT objects
                array_objects_gt.append(i)
                count_GT_objects_total += len(array_objects_gt)
                
                #get calculation values of target GT object with timestamp
                if i.obj_id == obj_id_target_gt:      
                                # timestamp       # GT object    
                    obj_gt = [timestamp_gt, msg.obj_list[index_counter]]
                    if category == "": # branch with one level in object list tree (e.g. 'obj_id')
                        array_values_gt.append(getattr(obj_gt[1], attribute))
                    else:
                        array_values_gt.append(getattr(getattr(obj_gt[1], category), attribute))
                else: index_counter += 1 # next object
            '''      
            #collect all GT objects
            for i in msg.obj_list:
                array_objects_gt.append(i)
                count_GT_objects_total += len(array_objects_gt)


            # finding concerning CAM object 
            # loop through CAM messages/frames to find message concerning to GT frame (in time) 
            for topic, msg, t in bag_cam.read_messages(topics=[TOPIC]):
                bag_cam.read_messages()
                timestamp_cam = (float)((t.__sub__(startTime_cam)).__str__()) / 1000000   #normed timestamp CAM in ms
                
                # time mapping: 
                # cam message in range +/- 500 ms from gt message
                if((timestamp_cam > (timestamp_gt - 500))
                and (timestamp_cam < (timestamp_gt + 500))):
                    
                    # position mapping: 
                    # loop through CAM objects in message --> proceed IoU evaluation
                    '''if value_cam_found == 0: '''
                    for i in msg.obj_list:
                        #collect all CAM objects
                        array_objects_cam.append(i)
                                        #index 0: timest.  #index 1: object
                        obj_cam = [timestamp_cam, i]
                      
                        ###
                        #   IoU testing: each CAM object compared to list of GT objects
                        #(evaluation, index_TP) = de.det_TP_FP_mm(array_objects_gt, obj_cam[1], IoU_threshold)
                        evaluation = 0
                        index_TP = 0
                        ###
                        
                        # TP (true positive) case - you got a match:
                        if evaluation == 0:
                            count_TP += 1
                            '''value_cam_found = 1'''
                            #break
                        
                        # FP (false positive) case:
                        elif evaluation == 1:
                            count_FP += 1
                        
                        # mm (mismatch) case:
                        elif evaluation == 2:
                            count_mm += 1
                  
                # test FN (false negative) case:
                for i in array_objects_gt:
                    
                    if de.isFN(i, array_objects_cam, IoU_threshold) == True:
                        count_FN += 1
                    
            '''          
            # PROBLEMATISCH ???
            if value_cam_found == 0:
                array_values_cam.append(last_value_cam)
                #print('insert last value: ')
                #print(last_val_cam)
            '''   
            array_precision.append(count_TP / (count_TP + count_FP))
            array_recall.append(count_TP / (count_TP + count_FN))
            array_TP.append(count_TP)
            array_FP.append(count_FP)
            array_FN.append(count_FN) 
            array_mm.append(count_mm)
            
        bag_gt.close()
        bag_cam.close()
        return (array_gttimestamps, array_TP, array_FP, array_FN, array_mm, array_precision, array_recall)
    '''
    @staticmethod
    def getmm(bagfile1, bagfile2, IoU_threshold):
        
        (timestamp, TP, FP, FN, mm) = getEvaluation(bagfile1, bagfile2, IoU_threshold)
        
        return (timestamp, mm)
    '''
    
    @staticmethod
    def getTP(bagfile1, bagfile2, IoU_threshold):
        
        (timestamp, TP, FP, FN, mm, precision, recall) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        
        return (timestamp, TP)
    
    @staticmethod
    def getFP(bagfile1, bagfile2, IoU_threshold):
        
        (timestamp, TP, FP, FN, mm, precision, recall) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        
        return (timestamp, FP)
    
    @staticmethod
    def getFN(bagfile1, bagfile2, IoU_threshold):
        
        (timestamp, TP, FP, FN, mm, precision, recall) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        
        return (timestamp, FN)
    
    @staticmethod
    def getPrecision(bagfile1, bagfile2, IoU_threshold):
        
        (timestamp, TP, FP, FN, mm, precision, recall) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        
        return (timestamp, precision)
    
    @staticmethod
    def getRecall(bagfile1, bagfile2, IoU_threshold):
    
        (timestamp, TP, FP, FN, mm, precision, recall) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        
        return (timestamp, recall)
        
    @staticmethod
    def getFPPI(bagfile1, bagfile2, IoU_threshold):
            
        (timestamp, TP, FP, FN, mm, precision, recall) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        
        FP_total = np.sum(FP)
        FPPI = FP_total / len(timestamp)  #len(timestamp) == nFrames_gt
        
        return FPPI
            
    @staticmethod
    def getMOTA(bagfile1, bagfile2, IoU_threshold):
        
        (timestamp, TP, FP, FN, mm, precision, recall) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
            
        FP_total = np.sum(FP)
        FN_total = np.sum(FN)
        mm_total = np.sum(mm)
        count_objects_GT_total = Rosbag_Analysis.getObjectCountTotal(bagfile1)

        MOTA = 1 - ((FN_total + FP_total + mm_total) / count_objects_GT_total)
    #@staticmethod
    #def get MOTP
    



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
