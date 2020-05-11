import rosbag
import genpy.rostime
import numpy as np
import matplotlib.pyplot as py

#bag_gt = rosbag.Bag('/home/christoph/git_ws/obj-lst-vis/src/tp3/bagfiles/Objektliste_2020-04-26-12-46-25.bag')
bag_gt = rosbag.Bag('/home/christoph/git_ws/obj-lst-vis/src/tp3/bagfiles/Objektliste.bag')

bag_cam = rosbag.Bag('/home/christoph/git_ws/obj-lst-vis/src/tp3/bagfiles/Objektliste_2020-04-26-12-47-33.bag')
#bag_cam = rosbag.Bag('/home/christoph/git_ws/obj-lst-vis/src/tp3/bagfiles/Objektliste.bag')


startTime_gt_raw= bag_gt.get_start_time()
startTime_gt = genpy.rostime.Time.from_sec(startTime_gt_raw)

startTime_cam_raw= bag_cam.get_start_time()
startTime_cam= genpy.rostime.Time.from_sec(startTime_cam_raw)

counter = 0
value_found = 0
last_val_cam = 0

array_values = []
array_timestamps_gt = []
array_timestamps_cam = []
array_msg_gt = []
array_msg_cam = []
diff = []

array_x_gt = []     #for plot only
array_x_cam= []  # for plot only
diff_time = [] # plot



for topic, msg, t in bag_gt.read_messages(topics=['/camera_obj']):
    bag_gt.read_messages()
    #array_msg_gt. append([(float)(t.__str__()) / 1000000, msg])
    array_msg_gt. append([(float)((t.__sub__(startTime_gt)).__str__()) / 1000000, msg])
    
    array_timestamps_gt.append((float)((t.__sub__(startTime_gt)).__str__()))
    array_x_gt.append(msg.obj_list[0].geometric.x)
    

for topic, msg, t in bag_cam.read_messages(topics=['/camera_obj']):
    bag_cam.read_messages()
    #array_msg_cam. append([(float)(t.__str__()) / 1000000, msg])
    array_msg_cam. append([(float)((t.__sub__(startTime_cam)).__str__()) / 1000000, msg])
    
    array_timestamps_cam.append((float)((t.__sub__(startTime_cam)).__str__()))
    array_x_cam.append(msg.obj_list[0].geometric.x)
#    array_timestamps_cam.append((float)((t.__sub__(startTime)).__str__()))

#print(array_timestamps_gt)

#array_timestamps_cam[0] = 0
#array_timestamps_cam[1] = 0
#array_timestamps_cam[2] = 0
#print(array_msg_gt[0][0])
#print(type(startTime_gt_raw)) #float
for i in range(0, len(array_msg_gt), 1):
    value_cam_found = 0
    
    for j in range(0, len(array_msg_cam), 1):
        
        #print('time_gt: ')
        #print(array_msg_gt[i][0])
        #print('time cam: ')
        #print(array_msg_cam[j][0])
        #print('value_gt: ')
        #print(array_msg_gt[i][1].obj_list[0].geometric.x)
        #print('value_cam: ')
        #print(array_msg_cam[j][1].obj_list[0].geometric.y)
        
       # if (array_msg_cam[j][0] > array_msg_gt[i][0]-500000000) and (array_msg_cam[j][0] < array_msg_gt[j][0]+500000000):
       
       ### this is it
        #if(((array_msg_cam[j][0] - startTime_cam_raw) > ((array_msg_gt[i][0] - startTime_gt_raw) - 500))
        #and ((array_msg_cam[j][0] - startTime_cam_raw) < ((array_msg_gt[i][0] - startTime_gt_raw) + 500))): # timestamps
        ###
        
        if(((array_msg_cam[j][0] ) > ((array_msg_gt[i][0] ) - 500))
        and ((array_msg_cam[j][0] ) < ((array_msg_gt[i][0] ) + 500))): # timestamps
        
            diff.append(abs(array_msg_gt[i][1].obj_list[0].geometric.x - array_msg_cam[j][1].obj_list[0].geometric.x))
            last_val_cam = array_msg_cam[j][1].obj_list[0].geometric.x
            
            value_cam_found = 1
            #print('time_gt: ')
            #print(array_msg_gt[i][0])
            #print('time cam: ')
            #print(array_msg_cam[j][0])
            
            
            print('value_gt: ')
            print(array_msg_gt[i][1].obj_list[0].geometric.x)
            print('value_cam: ')
            print(array_msg_cam[j][1].obj_list[0].geometric.x)
            print('diff: ')
            print(abs(array_msg_gt[i][1].obj_list[0].geometric.x - array_msg_cam[j][1].obj_list[0].geometric.x))
            
            
            
            break
        #else: 
        #    print('subtract last value: ')
        #    print(last_val_cam)
        #    diff.append(array_msg_gt[i][1].obj_list[0].geometric.x - last_val_cam)
        #    break
    if value_cam_found == 0:
        print('subtract last value: ')
        print(last_val_cam)
        diff.append(abs(array_msg_gt[i][1].obj_list[0].geometric.x - last_val_cam))
        

#print(len(diff))
#print(startTime_cam_raw-500000000)


py.plot(array_timestamps_gt, array_x_gt, '-b')
py.plot(array_timestamps_cam, array_x_cam, '-r')
py.plot(array_timestamps_gt, diff, '-k')        
py.show()     

