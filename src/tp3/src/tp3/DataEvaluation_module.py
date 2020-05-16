'''
    this module can compare to bag files and computes evaluation values, 
    such as IoU, TP, FP, FN, Precision, Recall, FPPI, MOTA, MOTP
'''

import shapely.geometry
import shapely.affinity
import numpy as np

def intersection(B_gt, B_pr):
    '''
        calculates the intersection of the given bounding boxes 
        B_gt (Ground Truth) and B_pr (Predicted)
    '''
    intersection = get_contour(B_gt).intersection(get_contour(B_pr))
    
    return intersection.area


def union(B_gt, B_pr):
    '''
        calculates the union of the given bounding boxes 
        B_gt (Ground Truth) and B_pr (Predicted)
    '''
    return area(B_gt) + area(B_pr) - intersection(B_gt, B_pr)


def iou(B_gt, B_pr):
    '''
        calculates the intersection over union of the given bounding boxes
        B_gt (Ground Truth) and B_pr (Predicted)
    '''
    return intersection(B_gt, B_pr) / union(B_gt, B_pr)

def det_TP_FP_mm(B_gt_list, B_pr, threshold):
    '''
        determines whether the given B_pr in this frame is a TP
        B_gt_list is a list of GT-Bounding Boxes in the given frame
        B_pr is a predicted Bounding Box which should be tested
        
        return value 1:
        0 -> given B_pr is a TP case
        1 -> given B_pr is a FN case
        2 -> given B_pr is a mm (mismatch) case 
        
        return value 2:
        if the case is TP there is a second return value
        which indicates the index of the GT-Object which 
        matched to the given B_pr
        
        if the case is not TP the second return value
        is None
    '''
    iou_list = []
    
    for B_gt in B_gt_list:
        iou = iou(B_gt, B_pr)
        iou_list.append(iou)
    
    max_iou = np.max(iou_list)
    
    if max_iou >= threshold: 
        # get the index:
        idx = iou_list.index(max_iou)
        
        # compare the classes:
        if B.pr.cls == B_gt_list[idx].cls:
            return (0, idx) # given B_pr is a TP case
        else:
            return (2, None) # given B_pr is a mm case
    
    else:
        return (1, None) # given B_pr is a FN case
    

def isFN(B_gt, B_pr_list, threshold):
    '''
        determines if there is a B_pr for the given B_gt
        B_gt_list is a list of GT-Bounding Boxes in the given frame
        B_pr is a predicted Bounding Box which should be tested
        
        return values:
        true -> no matching B_pr to this B_gt, a FN case is detected
        false -> no FN case
    '''
    iou_list = []
    
    for B_pr in B_pr_list:
        iou = iou(B_gt, B_pr)
        iou_list.append(iou)
    
    if np.max(iou_list) < threshold:
        return true
    
    else:
        return false    
    

def get_contour(B):
    l = B.dimension.length
    w = B.dimension.width
    c = shapely.geometry.box(-l / 2.0, -w / 2.0, l / 2.0, w / 2.0)
    rc = shapely.affinity.rotate(c, B.geometric.yaw)
    return shapely.affinity.translate(rc, B.geometric.x, B.geometric.y)  

def area(B):
    return get_contour(B).area  


# class BoundingBox2D():
#     pos_x = 0.0 # x value of the center
#     pos_y = 0.0 # y value of the center
#     length = 0.0 # length of the BB
#     width = 0.0 # width of the BB
#     yaw = 0.0 # yaw angle of the BB
#     cls = "" # class of the BB as String
#     
#     def __init__(self, x, y, l, w, yaw, cls):
#         self.pos_x = x
#         self.pos_y = y
#         self.length = l
#         self.width = w
#         self.yaw = yaw
#         self.cls = cls
#                
#     def get_contour(self):
#         l = self.length
#         w = self.width
#         c = shapely.geometry.box(-l / 2.0, -w / 2.0, l / 2.0, w / 2.0)
#         rc = shapely.affinity.rotate(c, self.yaw)
#         return shapely.affinity.translate(rc, self.pos_x, self.pos_y)
#     
#     def area(self):
#         return self.get_contour().area
# 
#     def intersection(self, other):
#         intersection = self.get_contour().intersection(other.get_contour())
#         return intersection.area 
#     
#     def union(self, other):
#         return self.area + other.area - self.intersection(other)
#     
#     def iou(self, other):
#         return self.intersection(other) / self.union(other)
    
    
    
