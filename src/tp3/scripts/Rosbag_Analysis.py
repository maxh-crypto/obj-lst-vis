'''
    This module can analyse Rosbag files and provide data for the post-processing GUI. 
    There are multiple methods that partly depend on each other. Some of them use 
    methods by ui.DataEvaluation_module.py
    
    author: Christoph Zach
    copyright by: Technische Hochschule Ingolstadt / Carissma
    date: June 2020
'''

import rosbag
import genpy.rostime
import numpy as np
import ui.DataEvaluation_module as de

TOPIC_main = 'objectlist'
TOPIC_GT = '/simulation'
TOPIC_CAM = '/camera_calculation'

TOPIC = TOPIC_main  # set name of ROS topic that should be analysed

class Rosbag_Analysis:
  
    @staticmethod
    def getObjectIDs(bagfile):
        '''
            delivers the object IDs (obj_id) contained in Rosbag file
            input: Rosbag file
            return: array of object IDs
        '''
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
        '''
            delivers the number of objects contained in one frame of the Rosbag file
            input: Rosbag file
            return: 
                array of time stamps
                array of object counts
                mean value of object counts (over whole Rosbag file)
                standard deviation of object counts (over whole Rosbag file)
        '''
        bag = rosbag.Bag(bagfile)
        array_objectCounts = []
        array_time = []
        
        startTime = genpy.rostime.Time.from_sec(bag.get_start_time())   # Rosbag start time
        
        for topic, msg, t in bag.read_messages(topics=[TOPIC]):
            count_objectsFrame = 0
            time = (float)((t.__sub__(startTime)).__str__()) / 1000000  # relative frame time to Rosbag start time
            
            for i in msg.obj_list:
                count_objectsFrame += 1
                
            array_time.append(time) 
            array_objectCounts.append(count_objectsFrame)
        
        meanValue = Rosbag_Analysis.calcMeanValue(array_objectCounts)   # mean value over whole Rosbag file
        standardDev = Rosbag_Analysis.calcStandardDeviation(array_objectCounts) # standard deviation over whole Rosbag file
        
        return (array_time, array_objectCounts, meanValue, standardDev)    
      
    @staticmethod
    def getObjectCountTotal(bagfile):
        '''
            delivers the number of objects summed up over all frames in Rosbag file
            input: Rosbag file
            return: number of objects summed up over all frames
        '''
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
        '''
            calculates the mean value of all values in input array
            input: array of values
            return: mean value
        '''
        if len(values) == 0:
            return float('nan')
        else: 
            meanValue = np.mean(values)
            return meanValue
   
    @staticmethod
    def calcStandardDeviation(values):
        '''
            calculates the standard deviation of all values in input array
            input: array of values
            return: standard deviation
        '''
        if len(values) == 0:
            return float('nan')
        else: 
            standardDev = np.std(values)
            return standardDev    
             
    @staticmethod
    def getRawData(bagfile, obj_id_target, category, attribute):
        '''
            delivers the values of a object attribute of all frames over the Rosbag file
            available object attributes: see ROS message 'Object_List'
            input: 
                Rosbag file,
                object ID (obj_id),
                category (level 1 in Object_List tree - e.g. "geometric" or "dimension")
                attribute (level 2 in Object_List tree - e.g. "x" or "width")
            return: 
                array of time stamps,
                array of attribute values,
                mean value of attribute values (over whole Rosbag file),
                standard deviation of attribute values (over whole Rosbag file)
        '''
        bag = rosbag.Bag(bagfile)
        
        startTime = genpy.rostime.Time.from_sec(bag.get_start_time())   # Rosbag start time

        array_values = []
        array_timestamps = []

        # loop through all frames in Rosbag file 
        for topic, msg, t in bag.read_messages(topics=[TOPIC]):
            index_counter = 0
            obj_id_index = 99
            
            time = (float)((t.__sub__(startTime)).__str__()) / 1000000  # frame time in milliseconds
            
            # loop through all objects in frame (message)
            # to find array index of target object ID
            for i in msg.obj_list:
                if i.obj_id == obj_id_target:
                    if category == "": # branch with one level in object list tree (e.g. 'obj_id')
                        array_values.append(getattr(msg.obj_list[index_counter], attribute))
                    else:
                        array_values.append(getattr(getattr(msg.obj_list[index_counter], category), attribute))
                    array_timestamps.append(time)  #timestamp in milliseconds
                    break
                else: index_counter += 1
        
        meanValue = Rosbag_Analysis.calcMeanValue(array_values)
        standardDev = Rosbag_Analysis.calcStandardDeviation(array_values)
        
        bag.close()
        return (array_timestamps, array_values, meanValue, standardDev) 
   
    @staticmethod
    def getAdvancedData(bagfile1, bagfile2, obj_id_target_gt, category, attribute, operation, IoU_threshold):
        '''
            delivers the values of advanced analysing methods in comparison of two Rosbag files
            available object attributes: see ROS message 'Object_List'
            available advanced analysing methods: 
                difference (extendable ...)
            input: 
                Rosbag file 1 (Ground-Truth data),
                Rosbag file 2 (sensor data),
                object ID (obj_id),
                category (level 1 in Object_List tree - e.g. "geometric" or "dimension"),
                attribute (level 2 in Object_List tree - e.g. "x" or "width"),
                operation (e.g. "difference"),
                threshold for IoU evaluation
            return: 
                array of time stamps,
                array of result values,
                mean value of result values 
                standard deviation of result values
        '''
        (timestamps, values_gt, values_cam) = Rosbag_Analysis.getCalculationValues(bagfile1, bagfile2, obj_id_target_gt, category, attribute, IoU_threshold)
        
        ### selected operation ###
        if operation == "difference":
            
            if len(values_gt) == 0 or len(values_cam) == 0:
                array_result = []
                
            else: 
                array_result = np.subtract(values_gt, values_cam)   # difference: value_GT - value_sensor
        
        # extendable: 
        # if operation == " ...":
        
        meanValue = Rosbag_Analysis.calcMeanValue(array_result)
        standardDev = Rosbag_Analysis.calcStandardDeviation(array_result)
        
        return (timestamps, array_result, meanValue, standardDev) 
   
    @staticmethod
    def timeMapping(frames_GT, frames_CAM):
        '''
            finds dedicated pairs of frames (GT and sensor data):
            To every sensor frame the last past GT frame is dedicated. 
            Time always in relative time to Rosbag start time
            visualization of schematic: see concerning IEEE paper
            
            input: 
                array of tuples - frame GT: (relative time stamp, message)
                array of tuples - frame sensor: (relative time stamp, message),
            return: 
                array of triples: (sensor time stamp, GT frame, sensor frame)
        '''
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
                
                #last iteration
                if frame_GT == frames_GT[-1]:
                    mapped_frames.append([time_CAM, latest_GT_frame[1], frame_CAM[1]])
                    
        return mapped_frames # array of triples of (timestamp_CAM (normed), frame_GT, frame_CAM)
                    
    @staticmethod
    def getFrames(bagfile1, bagfile2):
        '''
            Part 1:
            extracts all message frames out of GT data (Rosbag file 1) and
            sensor data (Rosbag file 2) and connects each frame with its specific time stamp.
            Every time stamp is relative frame time to the Rosbag start time.
            
            Part 2:
            executing timeMapping() to dedicate pairs of frames (GT and sensor data). 
            See timeMapping().
            
            input: 
                Rosbag file 1 (Ground-Truth data),
                Rosbag file 2 (sensor data),
            return: 
                array of triples: (sensor time stamp, GT frame, sensor frame)
        '''
        bag_gt = rosbag.Bag(bagfile1)
        bag_cam = rosbag.Bag(bagfile2)
        
        all_frames_GT = []
        all_frames_CAM = []
        
        # frame time
        startTime_gt = genpy.rostime.Time.from_sec(bag_gt.get_start_time())
        startTime_cam = genpy.rostime.Time.from_sec(bag_cam.get_start_time())
        
        # GT messages/frames: 
        # connect message with its relative time stamp in milliseconds in tuples
        for topic, msg, t in bag_gt.read_messages(topics=[TOPIC]):
            time = (float)((t.__sub__(startTime_gt)).__str__()) / 1000000
            all_frames_GT.append([time, msg])

        # sensor messages/frames: 
        # connect message with its relative time stamp in milliseconds in tuples
        for topic, msg, t in bag_cam.read_messages(topics=[TOPIC]):
            time = (float)((t.__sub__(startTime_cam)).__str__()) / 1000000
            all_frames_CAM.append([time, msg])

        #time mapping: finding matching GT frame to every sensor frame
        #mapped frames =  array of triples: (timestamp_CAM (normed), mapped_frame_GT, mapped_frame_CAM)
        mapped_frames = Rosbag_Analysis.timeMapping(all_frames_GT, all_frames_CAM)
        
        bag_gt.close()
        bag_cam.close()
        return mapped_frames
        
    @staticmethod
    def getCalculationValues(bagfile1, bagfile2, obj_id_target_gt, category, attribute, IoU_threshold):
        '''
            provides value arrays for calculations in getAdvancedData().
            Uses values of selected object ID (according to GT data) if a
            detected sensor object can be matched (only True Positive cases are used).
            
            input: 
                Rosbag file 1 (Ground-Truth data),
                Rosbag file 2 (sensor data),
                object ID (GT: obj_id),
                category (level 1 in Object_List tree - e.g. "geometric" or "dimension"),
                attribute (level 2 in Object_List tree - e.g. "x" or "width"),
                threshold for IoU evaluation
            return: 
                array of time stamps (sensor)
                array of values GT
                array of values sensor
        '''
        # frame mapping
        mapped_frames = Rosbag_Analysis.getFrames(bagfile1, bagfile2)
        
        timestamps_CAM = []
        values_GT = []
        values_CAM = []
        
        # analyse every pair of frames (sensor message and concerning GT message)
        for frame in mapped_frames:
        
            objectsInFrame_GT = []
            objectsInFrame_CAM = []
            triple_count = -1
            
            #collect all GT objects in current GT frame
            for object_gt in frame[1].obj_list:
                objectsInFrame_GT.append(object_gt)
                
            #collect all CAM objects in current sensor frame
            for object_cam in frame[2].obj_list:
                objectsInFrame_CAM.append(object_cam)
        
            ###
            ### proceeding IoU evaluation
            evaluations = de.det_TP_FP_mm(objectsInFrame_GT, objectsInFrame_CAM, IoU_threshold)
            ###

            # analysing IoU evaluation results
            for triple in evaluations:
                triple_count += 1

                # TP (True Positive) case (enumeration value 0) for selected GT object - you got a match:   
                if triple[0] == 0 and objectsInFrame_GT[triple[1]].obj_id == obj_id_target_gt:
                
                    timestamps_CAM.append(frame[0]) # sensor time stamp (relative)
                    
                    # fetch attribute value of object
                    if category == "": # branch with one level in object list tree (e.g. 'obj_id')
                        values_GT.append(getattr(objectsInFrame_GT[triple[1]], attribute))
                        values_CAM.append(getattr(objectsInFrame_CAM[triple_count], attribute))
                    else:
                        values_GT.append(getattr(getattr(objectsInFrame_GT[triple[1]], category), attribute))
                        values_CAM.append(getattr(getattr(objectsInFrame_CAM[triple_count], category), attribute))
                    break
        
        # no matches for selected GT object detected: no Ground-Truth data available
        if len(values_GT) == 0:
            print("Error in Rosbag_Analysis.py: No object matches detected. Could not get calculation values.")
        
        return (timestamps_CAM, values_GT, values_CAM)      
     
    @staticmethod
    def getEvaluation(bagfile1, bagfile2, IoU_threshold):
        '''
            provides value arrays for the evaluation of Quality-of-Service parameter of a sensor detection
            in comparison to Ground-Truth data.
            
            input: 
                Rosbag file 1 (Ground-Truth data),
                Rosbag file 2 (sensor data),
                threshold for IoU evaluation
            return: 
                array of time stamps (sensor)
                array of number of TP cases
                array of number of FP cases
                array of number of FN cases
                array of number of mm cases
                array of values of precision
                array of values of recall 
                array of IoU evaluation values (only TP cases regarded)
        '''
        # frame mapping
        mapped_frames = Rosbag_Analysis.getFrames(bagfile1, bagfile2)
        
        array_timestamps = []
        array_TP = []
        array_IoU_values_TP = []
        array_FP = []
        array_FN = []
        array_mm = []
        array_precision = []
        array_recall = []
        
        # analyse every pair of frames (sensor message and concerning GT message)        
        for frame in mapped_frames:
        
            objectsInFrame_GT = []
            objectsInFrame_CAM = []
            
            count_TP = 0.0
            count_FP = 0.0
            count_mm = 0.0
            count_FN = 0.0
            sum_IoU_values = 0.0
            
            array_timestamps.append(frame[0])   # sensor time stamp (relative)
            
            #collect all GT objects in current frame
            for object_gt in frame[1].obj_list:
                objectsInFrame_GT.append(object_gt)

            # collect all CAM objects in current frame
            for object_cam in frame[2].obj_list:
                objectsInFrame_CAM.append(object_cam)
           
            ###
            ### proceeding IoU evaluation
            evaluations = de.det_TP_FP_mm(objectsInFrame_GT, objectsInFrame_CAM, IoU_threshold)
            ###

            # analysing IoU evaluation results
            for triple in evaluations:
            
                # TP (True Positive) case - you got a match:
                if triple[0] == 0:
                    count_TP += 1
                    sum_IoU_values += triple[2]
                
                # FP (False Positive) case:
                elif triple[0] == 1:
                    count_FP += 1
                
                # mm (mismatch) case:
                elif triple[0] == 2:
                    count_mm += 1
          
            # evaluating FN (False Negative) case:                    
            evaluations_FN = de.isFN(objectsInFrame_GT, objectsInFrame_CAM, IoU_threshold)
           
            # counting FN cases
            for i in evaluations_FN:
                if i == True:
                    count_FN += 1
            
            # collecting return values for each pair of frame
            array_TP.append(count_TP)
            array_FP.append(count_FP)
            array_FN.append(count_FN) 
            array_mm.append(count_mm)
            array_IoU_values_TP.append(sum_IoU_values)
            
            # calculating value of precision (per frame)
            if (count_TP + count_FP) == 0:
                array_precision.append(0)
            else: array_precision.append(count_TP / (count_TP + count_FP))
            
            # calculating value of recall (per frame)
            if (count_TP + count_FN) == 0:
                array_recall.append(0)
            else: array_recall.append(count_TP / (count_TP + count_FN))
            
        return (array_timestamps, array_TP, array_FP, array_FN, array_mm, array_precision, array_recall, array_IoU_values_TP)     
    
    @staticmethod
    def getTP(bagfile1, bagfile2, IoU_threshold):
        '''
            delivers the number of True Positive (TP) cases calculated by 
            IoU (Intersection over Union) algorithm
            input: 
                Rosbag file 1 (Ground-Truth data),
                Rosbag file 2 (sensor data),
                threshold for IoU evaluation
            return:
                array of time stamps,
                array of number of TP cases per frame,
                mean value of TP cases
                standard deviation of TP cases
        '''
        (timestamp, TP, FP, FN, mm, precision, recall, IoU_values_TP) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        meanValue = Rosbag_Analysis.calcMeanValue(TP)
        standardDev = Rosbag_Analysis.calcStandardDeviation(TP)
        
        return (timestamp, TP, meanValue, standardDev)
    
    @staticmethod
    def getFP(bagfile1, bagfile2, IoU_threshold):
        '''
            delivers the number of False Positive (FP) cases calculated by 
            IoU (Intersection over Union) algorithm
            input: 
                Rosbag file 1 (Ground-Truth data),
                Rosbag file 2 (sensor data),
                threshold for IoU evaluation
            return:
                array of time stamps,
                array of number of FP cases per frame,
                mean value of FP cases
                standard deviation of FP cases
        '''
        (timestamp, TP, FP, FN, mm, precision, recall, IoU_values_TP) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        meanValue = Rosbag_Analysis.calcMeanValue(FP)
        standardDev = Rosbag_Analysis.calcStandardDeviation(FP)
        
        return (timestamp, FP, meanValue, standardDev)
        
    @staticmethod
    def getmm(bagfile1, bagfile2, IoU_threshold):
        '''
            delivers the number of mismatch (mm) cases calculated by 
            IoU (Intersection over Union) algorithm
            input: 
                Rosbag file 1 (Ground-Truth data),
                Rosbag file 2 (sensor data),
                threshold for IoU evaluation
            return:
                array of time stamps,
                array of number of mm cases per frame,
                mean value of mm cases
                standard deviation of mm cases
        '''
        (timestamp, TP, FP, FN, mm, precision, recall, IoU_values_TP) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        meanValue = Rosbag_Analysis.calcMeanValue(mm)
        standardDev = Rosbag_Analysis.calcStandardDeviation(mm)
        
        return (timestamp, mm, meanValue, standardDev)
    
    @staticmethod
    def getFN(bagfile1, bagfile2, IoU_threshold):
        '''
            delivers the number of False Negative (FN) cases calculated by 
            IoU (Intersection over Union) algorithm
            input: 
                Rosbag file 1 (Ground-Truth data),
                Rosbag file 2 (sensor data),
                threshold for IoU evaluation
            return:
                array of time stamps,
                array of number of FN cases per frame,
                mean value of FN cases
                standard deviation of FN cases
        '''
        (timestamp, TP, FP, FN, mm, precision, recall, IoU_values_TP) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        meanValue = Rosbag_Analysis.calcMeanValue(FN)
        standardDev = Rosbag_Analysis.calcStandardDeviation(FN)
        
        return (timestamp, FN, meanValue, standardDev)
    
    @staticmethod
    def getPrecision(bagfile1, bagfile2, IoU_threshold):
        '''
            delivers the value of precision calculated by getEvaluation()
            input: 
                Rosbag file 1 (Ground-Truth data),
                Rosbag file 2 (sensor data),
                threshold for IoU evaluation
            return:
                array of time stamps,
                array of values of precision per frame,
                mean value of precision values
                standard deviation of precision values
        '''
        (timestamp, TP, FP, FN, mm, precision, recall, IoU_values_TP) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        meanValue = Rosbag_Analysis.calcMeanValue(precision)
        standardDev = Rosbag_Analysis.calcStandardDeviation(precision)
        
        return (timestamp, precision, meanValue, standardDev)
    
    @staticmethod
    def getRecall(bagfile1, bagfile2, IoU_threshold):
        '''
            delivers the value of recall calculated by getEvaluation()
            input: 
                Rosbag file 1 (Ground-Truth data),
                Rosbag file 2 (sensor data),
                threshold for IoU evaluation
            return:
                array of time stamps,
                array of values of recall per frame,
                mean value of recall values
                standard deviation of recall values
        '''
        (timestamp, TP, FP, FN, mm, precision, recall, IoU_values_TP) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        meanValue = Rosbag_Analysis.calcMeanValue(recall)
        standardDev = Rosbag_Analysis.calcStandardDeviation(recall)
        
        return (timestamp, recall, meanValue, standardDev)
        
    @staticmethod
    def getFPPI(bagfile1, bagfile2, IoU_threshold):
        '''
            calculates the value of False Positive Per Image (FPPI) over whole Rosbag file
            input: 
                Rosbag file 1 (Ground-Truth data),
                Rosbag file 2 (sensor data),
                threshold for IoU evaluation
            return:
                value FPPI
        '''    
        (timestamp, TP, FP, FN, mm, precision, recall, IoU_values_TP) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        
        FP_total = np.sum(FP)
        
        if len(timestamp) == 0:
            FPPI = float('nan')
        
        else:
            FPPI = FP_total / len(timestamp)  # len(timestamp) == nFrames in sensor data
        
        return FPPI
            
    @staticmethod
    def getMOTA(bagfile1, bagfile2, IoU_threshold):
        '''
            calculates the value of Multiple Object Tracking Accuracy (MOTA) over whole Rosbag file
            input: 
                Rosbag file 1 (Ground-Truth data),
                Rosbag file 2 (sensor data),
                threshold for IoU evaluation
            return:
                value MOTA
        '''  
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
        '''
            calculates the value of Multiple Object Tracking Precision (MOTP) over whole Rosbag file
            input: 
                Rosbag file 1 (Ground-Truth data),
                Rosbag file 2 (sensor data),
                threshold for IoU evaluation
            return:
                value MOTP
        ''' 
        (timestamp, TP, FP, FN, mm, precision, recall, IoU_values_TP) = Rosbag_Analysis.getEvaluation(bagfile1, bagfile2, IoU_threshold)
        
        d_ti = np.sum(IoU_values_TP)
        TP_total = np.sum(TP)
        
        if TP_total == 0 :
            MOTP = 0
        else:
            MOTP = d_ti / TP_total
        
        return MOTP


    ### following: example method - DEPRECATED ###
    ### how to get all values of geometric.x of object with ID 1 in a Rosbag file ###
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
            array_timestamps.append((float)((t.__sub__(startTime)).__str__()) / 1000000)  # time stamp in milliseconds
        
        bag.close()
        return (array_timestamps, array_x) 
