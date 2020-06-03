import rosbag
import genpy.rostime
import numpy as np
import ui.DataEvaluation_module as de

# These functions can be called by the file "Rosbag_Analysis_main.py" #

TOPIC_main = 'objectlist'
TOPIC_GT = '/simulation'
TOPIC_CAM = '/camera_calculation'

TOPIC = TOPIC_main

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
      
    @staticmethod
    def getObjectCountPerFrame(bagfile):
        bag = rosbag.Bag(bagfile)
        array_objectCounts = []
        array_time = []
        
        startTime = genpy.rostime.Time.from_sec(bag.get_start_time())
        
        for topic, msg, t in bag.read_messages(topics=[TOPIC]):
            count_objectsFrame = 0
            time = (float)((t.__sub__(startTime)).__str__()) / 1000000
            
            for i in msg.obj_list:
                count_objectsFrame += 1
                
            array_time.append(time) 
            array_objectCounts.append(count_objectsFrame)
        
        meanValue = Rosbag_Analysis.calcMeanValue(array_objectCounts)
        standardDev = Rosbag_Analysis.calcStandardDeviation(array_objectCounts)
        
        return (array_time, array_objectCounts, meanValue, standardDev)    
      
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
   
    @staticmethod
    def calcMeanValue(values):
        
        if len(values) == 0:
            return nan
        else: 
            meanValue = np.mean(values)
            return meanValue
   
    @staticmethod
    def calcStandardDeviation(values):
        
        if len(values) == 0:
            return nan
        else: 
            standardDev = np.std(values)
            return standardDev    
             
    @staticmethod
    def getRawData(bagfile, obj_id_target, category, attribute):
        
        bag = rosbag.Bag(bagfile)
        
        startTime = genpy.rostime.Time.from_sec(bag.get_start_time())

        array_values = []
        array_timestamps = []

        # loop through all messages in bagfile
        for topic, msg, t in bag.read_messages(topics=[TOPIC]):
            index_counter = 0
            obj_id_index = 99
            
            time = (float)((t.__sub__(startTime)).__str__()) / 1000000
            
            # loop through all objects in message
            # to find array index of target object ID
            for i in msg.obj_list:
                if i.obj_id == obj_id_target:
                    if category == "": # branch with one level in object list tree (e.g. 'obj_id')
                        array_values.append(getattr(msg.obj_list[index_counter], attribute))
                    else:
                        array_values.append(getattr(getattr(msg.obj_list[index_counter], category), attribute))
                    array_timestamps.append(time)  #timestamp in milli seconds
                    break
                else: index_counter += 1
        
        meanValue = Rosbag_Analysis.calcMeanValue(array_values)
        standardDev = Rosbag_Analysis.calcStandardDeviation(array_values)
        
        bag.close()
        return (array_timestamps, array_values, meanValue, standardDev) 
   
    @staticmethod
    def getAdvancedData(bagfile1, bagfile2, obj_id_target_gt, category, attribute, operation, IoU_threshold):
    
        (timestamps, values_gt, values_cam) = Rosbag_Analysis.getCalculationValues(bagfile1, bagfile2, obj_id_target_gt, category, attribute, IoU_threshold)
        
        ### operation ###
        if operation == "difference":
            
            if len(values_gt) == 0 or len(values_cam) == 0:
                array_result = []
                
            else: 
                array_result = np.subtract(values_gt, values_cam)

        meanValue = Rosbag_Analysis.calcMeanValue(array_result)
        standardDev = Rosbag_Analysis.calcStandardDeviation(array_result)
        
        return (timestamps, array_result, meanValue, standardDev) 
   
    @staticmethod
    def timeMapping(frames_GT, frames_CAM):    # frames = array of tuples of (normed timestamps, messages)
        
        mapped_frames = []
        
        for frame_CAM in frames_CAM:
            latest_GT_frame = 0
            time_CAM = frame_CAM[0]
            
            for frame_GT in frames_GT:
                time_GT = frame_GT[0]
                
                if (time_GT > time_CAM):                
                    mapped_frames.append([time_CAM, latest_GT_frame[1], frame_CAM[1]])
                    break
                else:
                    latest_GT_frame = frame_GT
                
                #last circle
                if frame_GT == frames_GT[-1]:
                    mapped_frames.append([time_CAM, latest_GT_frame[1], frame_CAM[1]])
                    
        return mapped_frames # array of triples of (timestamp_CAM (normed), frame_GT, frame_CAM)
                    
    @staticmethod
    def getFrames(bagfile1, bagfile2):
        
        bag_gt = rosbag.Bag(bagfile1)
        bag_cam = rosbag.Bag(bagfile2)
        
        all_frames_GT = []
        all_frames_CAM = []
        
        #time
        startTime_gt = genpy.rostime.Time.from_sec(bag_gt.get_start_time())
        startTime_cam = genpy.rostime.Time.from_sec(bag_cam.get_start_time())
        
         # collect GT messages/frames
        for topic, msg, t in bag_gt.read_messages(topics=[TOPIC]):
            time = (float)((t.__sub__(startTime_gt)).__str__()) / 1000000
            all_frames_GT.append([time, msg])

        # collect CAM messages/frames
        for topic, msg, t in bag_cam.read_messages(topics=[TOPIC]):
            time = (float)((t.__sub__(startTime_cam)).__str__()) / 1000000
            all_frames_CAM.append([time, msg])

        #time mapping: match GT frame to each CAM frame (once for all frames)
        #mapped frames =  array of triples: (timestamp_CAM (normed), mapped_frame_GT, mapped_frame_CAM)
        mapped_frames = Rosbag_Analysis.timeMapping(all_frames_GT, all_frames_CAM)
        
        bag_gt.close()
        bag_cam.close()
        return mapped_frames
        
    @staticmethod
    def getCalculationValues(bagfile1, bagfile2, obj_id_target_gt, category, attribute, IoU_threshold):
        
        mapped_frames = Rosbag_Analysis.getFrames(bagfile1, bagfile2)
        
        timestamps_CAM = []
        values_GT = []
        values_CAM = []
            
        for frame in mapped_frames:
        
            objectsInFrame_GT = []
            objectsInFrame_CAM = []
            triple_count = -1
            
            #collect all GT objects in frame
            for object_gt in frame[1].obj_list:
                objectsInFrame_GT.append(object_gt)
                
            #collect all CAM objects in frame
            for object_cam in frame[2].obj_list:
                objectsInFrame_CAM.append(object_cam)
        
            ###
            #IoU testing - position mapping
            evaluations = de.det_TP_FP_mm(objectsInFrame_GT, objectsInFrame_CAM, IoU_threshold)
            ###

            # analysing evaluation results
            for triple in evaluations:
                triple_count += 1

                # TP (true positive) case - you got a match:   
                if triple[0] == 0 and objectsInFrame_GT[triple[1]].obj_id == obj_id_target_gt:
                
                    timestamps_CAM.append(frame[0])
                    
                    if category == "": # branch with one level in object list tree (e.g. 'obj_id')
                        values_GT.append(getattr(objectsInFrame_GT[triple[1]], attribute))
                        values_CAM.append(getattr(objectsInFrame_CAM[triple_count], attribute))
                    else:
                        values_GT.append(getattr(getattr(objectsInFrame_GT[triple[1]], category), attribute))
                        values_CAM.append(getattr(getattr(objectsInFrame_CAM[triple_count], category), attribute))
                    break
                        
        if len(values_GT) == 0:
            print("Error in Rosbag_Analysis.py: No object matches detected. Could not get calculation values.")
        
        return (timestamps_CAM, values_GT, values_CAM)      
     
    @staticmethod
    def getEvaluation(bagfile1, bagfile2, IoU_threshold):
        
        mapped_frames = Rosbag_Analysis.getFrames(bagfile1, bagfile2)
        
        array_timestamps = []
        array_TP = []
        array_IoU_values_TP = []
        array_FP = []
        array_FN = []
        array_mm = []
        array_precision = []
        array_recall = []
                    
        for frame in mapped_frames:
        
            objectsInFrame_GT = []
            objectsInFrame_CAM = []
            
            count_TP = 0.0
            count_FP = 0.0
            count_mm = 0.0
            count_FN = 0.0
            sum_IoU_values = 0.0
            
            array_timestamps.append(frame[0])
            
            #collect all GT objects in frame
            for object_gt in frame[1].obj_list:
                objectsInFrame_GT.append(object_gt)

            # collect all CAM objects in frame
            for object_cam in frame[2].obj_list:
                objectsInFrame_CAM.append(object_cam)
           
            ###
            #IoU testing - position mapping
            evaluations = de.det_TP_FP_mm(objectsInFrame_GT, objectsInFrame_CAM, IoU_threshold)
            ###

            # analysing evaluation results
            for triple in evaluations:
            
                # TP (true positive) case - you got a match:
                if triple[0] == 0:
                    count_TP += 1
                    sum_IoU_values += triple[2]
                
                # FP (false positive) case:
                elif triple[0] == 1:
                    count_FP += 1
                
                # mm (mismatch) case:
                elif triple[0] == 2:
                    count_mm += 1
          
            # test FN (false negative) case:                    
            evaluations_FN = de.isFN(objectsInFrame_GT, objectsInFrame_CAM, IoU_threshold)
            
            for i in evaluations_FN:
                if i == True:
                    count_FN += 1
            
            # add values for each frame
            array_TP.append(count_TP)
            array_FP.append(count_FP)
            array_FN.append(count_FN) 
            array_mm.append(count_mm)
            array_IoU_values_TP.append(sum_IoU_values)
            
            if (count_TP + count_FP) == 0:
                array_precision.append(0)
            else: array_precision.append(count_TP / (count_TP + count_FP))
            
            if (count_TP + count_FN) == 0:
                array_recall.append(0)
            else: array_recall.append(count_TP / (count_TP + count_FN))
            
            array_IoU_values_TP.append(sum_IoU_values)
            
                   #timestamps CAM
        return (array_timestamps, array_TP, array_FP, array_FN, array_mm, array_precision, array_recall, array_IoU_values_TP)     
    
    @staticmethod
    def getTP(bagfile1, bagfile2, IoU_threshold):
        
        (timestamp, TP, FP, FN, mm, precision, recall, IoU_values_TP) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        meanValue = Rosbag_Analysis.calcMeanValue(TP)
        standardDev = Rosbag_Analysis.calcStandardDeviation(TP)
        
        return (timestamp, TP, meanValue, standardDev)
    
    @staticmethod
    def getFP(bagfile1, bagfile2, IoU_threshold):
        
        (timestamp, TP, FP, FN, mm, precision, recall, IoU_values_TP) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        meanValue = Rosbag_Analysis.calcMeanValue(FP)
        standardDev = Rosbag_Analysis.calcStandardDeviation(FP)
        
        return (timestamp, FP, meanValue, standardDev)
        
    @staticmethod
    def getmm(bagfile1, bagfile2, IoU_threshold):
        
        (timestamp, TP, FP, FN, mm, precision, recall, IoU_values_TP) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        meanValue = Rosbag_Analysis.calcMeanValue(mm)
        standardDev = Rosbag_Analysis.calcStandardDeviation(mm)
        
        return (timestamp, mm, meanValue, standardDev)
    
    @staticmethod
    def getFN(bagfile1, bagfile2, IoU_threshold):
        
        (timestamp, TP, FP, FN, mm, precision, recall, IoU_values_TP) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        meanValue = Rosbag_Analysis.calcMeanValue(FN)
        standardDev = Rosbag_Analysis.calcStandardDeviation(FN)
        
        return (timestamp, FN, meanValue, standardDev)
    
    @staticmethod
    def getPrecision(bagfile1, bagfile2, IoU_threshold):
        
        (timestamp, TP, FP, FN, mm, precision, recall, IoU_values_TP) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        meanValue = Rosbag_Analysis.calcMeanValue(precision)
        standardDev = Rosbag_Analysis.calcStandardDeviation(precision)
        
        return (timestamp, precision, meanValue, standardDev)
    
    @staticmethod
    def getRecall(bagfile1, bagfile2, IoU_threshold):
    
        (timestamp, TP, FP, FN, mm, precision, recall, IoU_values_TP) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        meanValue = Rosbag_Analysis.calcMeanValue(recall)
        standardDev = Rosbag_Analysis.calcStandardDeviation(recall)
        
        return (timestamp, recall, meanValue, standardDev)
        
    @staticmethod
    def getFPPI(bagfile1, bagfile2, IoU_threshold):
            
        (timestamp, TP, FP, FN, mm, precision, recall, IoU_values_TP) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        
        FP_total = np.sum(FP)
        
        if len(timestamp) == 0:
            FPPI = nan
        
        else:
            FPPI = FP_total / len(timestamp)  #len(timestamp) == nFrames
        
        return FPPI
            
    @staticmethod
    def getMOTA(bagfile1, bagfile2, IoU_threshold):
        
        (timestamp, TP, FP, FN, mm, precision, recall, IoU_values_TP) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
            
        FP_total = np.sum(FP)
        FN_total = np.sum(FN)
        mm_total = np.sum(mm)
        count_objects_GT_total = Rosbag_Analysis.getObjectCountTotal(bagfile1)

        if count_objects_GT_total == 0:
            MOTA = 0
        else:
            MOTA = 1 - ((FN_total + FP_total + mm_total) / count_objects_GT_total)
        
        return MOTA
    
    @staticmethod
    def getMOTP(bagfile1, bagfile2, IoU_threshold):
        
        (timestamp, TP, FP, FN, mm, precision, recall, IoU_values_TP) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        
        d_ti = np.sum(IoU_values_TP)
        TP_total = np.sum(TP)
        
        if TP_total == 0 :
            MOTP = 0
        else:
            MOTP = d_ti / TP_total
        
        return MOTP


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
        
        bag.close()
        return (array_timestamps, array_x) 
