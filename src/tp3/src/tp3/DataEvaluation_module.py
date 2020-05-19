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
        
        first return value:
        0 -> given B_pr is a TP case
        1 -> given B_pr is a FP case
        2 -> given B_pr is a mm (mismatch) case 
        
        second return value:
        if the case is TP there is a second return value
        which indicates the index of the GT-Object which 
        matched to the given B_pr
        
        if the case is not TP the second return value
        is 'None'
    '''
    iou_list = []
    
    for B_gt in B_gt_list:
        iou_val = iou(B_gt, B_pr)
        iou_list.append(iou_val)
    
    max_iou = np.max(iou_list)
    
    if max_iou >= threshold: 
        # get the index:
        idx = iou_list.index(max_iou)
        
        # compare the classes:
        if getClass(B_pr) == getClass(B_gt_list[idx]):
            return (0, idx) # given B_pr is a TP case
        else:
            return (2, idx) # given B_pr is a mm case
    
    else:
        return (1, None) # given B_pr is a FP case
    
    
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
        iou_val = iou(B_gt, B_pr)
        iou_list.append(iou_val)
    
    if np.max(iou_list) < threshold:
        return True
    
    else:
        return False    
    

def evaluateObjects(B_gt_list, B_pr_list, threshold):
    '''
        returns a matrix with
    '''
    
    
def get_contour(B):
    l = B.dimension.length
    w = B.dimension.width
    c = shapely.geometry.box(-l / 2.0, -w / 2.0, l / 2.0, w / 2.0)
    rc = shapely.affinity.rotate(c, B.geometric.yaw)
    return shapely.affinity.translate(rc, B.geometric.x, B.geometric.y)  


def area(B):
    return get_contour(B).area  


def getClass(B):

    temp_prop = 0
    result = ""
    #tmp includes all Attributes of the message Classification
    tmp = [a for a in dir(B.classification) if not a.startswith('__') and not a.startswith('_') and not callable(getattr(B.classification,a))]
    

    for i in range(len(tmp)):
        if(getattr(B.classification, tmp[i]) > temp_prop ):
            temp_prop = getattr(B.classification, tmp[i])
            result = tmp[i]
    return (result) # return value is the name of the class whith the highest probability


def genIouMat(B_gt_list, B_pr_list):
    '''
        generates a matrix of all iou values
        rows: Cam-Objects
        columns: GT-Objects
    '''
    row_count = len(B_pr_list)
    col_count = len(B_gt_list)
    
    iou_mat = np.zeros((row_count, col_count))
    
    cur_row_idx = 0
    
    # fill iou matrix
    for row in range(row_count):
        for col in range(col_count):
            iou_val = iou(B_gt_list(col), B_pr_list(row))
            iou_mat[row][col] = iou_val

    return iou_mat


def getTPs(B_gt_list, B_pr_list, threshold):
    '''
        returns a list of tupels with the indices of matching objects
        like (gt_index, pr_index)
        the indices refer to the indices in the passed lists
    '''
    tp_list = []
    
    iou_mat = genIouMat(B_gt_list, B_pr_list)
    
    for row in iou_mat:
        max_iou = np.max(row) # get the maximum iou of all B_gts for this B_pr
        
        if max_iou < threshold: 
            continue
        
        # get the indices:
        row_idx = iou_mat.index(row)
        col_idx = row.index(max_iou)
            
        # check the classes
        if getClass(B_pr_list[row_idx]) != getClass(B_gt_list[col_idx]):
            # missmatch found
            continue
        
        # if there is another match for this B_gt
        # this B_pr would be a FP
        max_in_col = np.max(iou_mat[row_idx])
        
        if max_in_col != max_iou: 
            continue

        # otherwise it is a TP
        tp_list.append(col_idx , row_idx)
        
    return tp_list

def getMMs(B_gt_list, B_pr_list, threshold):
    '''
        returns a list of indices of those pr_Objects 
        which are accounted as a mismatch
        the indices refer to the indices in the passed B_pr_list
    '''
    mm_list = []
    
    iou_mat = genIouMat(B_gt_list, B_pr_list)
    
    for row in iou_mat:
        max_iou = np.max(row) # get the maximum iou of all B_gts for this B_pr
        
        if max_iou < threshold: 
            continue
        
        # get the indices:
        row_idx = iou_mat.index(row)
        col_idx = row.index(max_iou)
            
        # check the classes
        if getClass(B_pr_list[row_idx]) == getClass(B_gt_list[col_idx]):
            continue
        
        # if there is another match for this B_gt
        # this B_pr would be a FP
        max_in_col = np.max(iou_mat[row_idx])
        
        if max_in_col != max_iou: 
            continue

        # otherwise it is a mm
        mm_list.append(row_idx)
        
    return mm_list


def getFPs(B_gt_list, B_pr_list, threshold):
    '''
        returns a list of indices of those pr_Objects 
        which are accounted as a FP
        the indices refer to the indices in the passed B_pr_list
    '''
    
    fp_list = []
    
    iou_mat = genIouMat(B_gt_list, B_pr_list)
    
    # two cases:
    # 1. there is no B_gt for which the iou is higher than threshold
    # 2. there is a B_gt for which the iou is higher than threshold,
    #     but iou is higher with another B_pr
    
    for row in iou_mat:
        row_idx = iou_mat.index(row)
        
        max_iou = np.max(row) # get the maximum iou of all B_gts for this B_pr
        
        if max_iou < threshold: # case 1
            # fp found
            fp_list.append(row_idx)
            continue
        
        ## case 2
        if max_iou < np.max(iou_mat[row_index])
    
    return fp_list
