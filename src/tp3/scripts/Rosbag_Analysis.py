import rosbag
import genpy.rostime

# These functions can be called by the file "Rosbag_Analysis_main.py" #

class Rosbag_Analysis:
    
    ### method for getting count of objects contained in rosbag ###
    
    @staticmethod
    def getObjectCount(bagfile):
        
        bag = rosbag.Bag(bagfile)
        count_max = 0
        
        for topic, msg, t in bag.read_messages(topics=['/camera_obj']):
            count = 0
            for i in msg.obj_list:
                count += 1
            
            if(count > count_max):
                count_max = count
        
        bag.close()
        return count_max
    
    
    ### method for getting array of raw values of all frames in a rosbag ###
    ### generic implementation ###
    
    @staticmethod
    def getRawData(bagfile, obj_id, category, attribute):
        
        bag = rosbag.Bag(bagfile)
		       
        startTimeRaw= bag.get_start_time()
        startTime = genpy.rostime.Time.from_sec(startTimeRaw)

        counter = 0
        array_values = []
        array_timestamps = []

        for topic, msg, t in bag.read_messages(topics=['/camera_obj']):
            counter += 1
            bag.read_messages()
            
            array_values.append(getattr(getattr(msg.obj_list[obj_id], category), attribute))
            
            array_timestamps.append((float)((t.__sub__(startTime)).__str__()) / 1000000)  #appends timestamp in milli seconds
           
        #print('Number of sequences: ')     #sequence = count of frames
        #print(counter)
        
        bag.close()
        return (array_timestamps, array_values)
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
        
        for topic, msg, t in bag.read_messages(topics=["/camera_obj"]):
            counter += 1
            bag.read_messages()
            
            array_x.append(msg.obj_list[obj_id].geometric.x)
            array_timestamps.append((float)((t.__sub__(startTime)).__str__()) / 1000000)  #appends timestamp in milli seconds
           
        #print('Number of sequences: ')     #sequence = count of frames
        #print(counter)
        
        bag.close()
        return (array_timestamps, array_x)
    ######################   
