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


def get_contour(B):
#     print(str(B.geometric.x) + ', ' + str(B.geometric.y))
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


def det_TP_FP_mm(B_gt_list, B_pr_list, threshold):
    '''
        determines whether the B_prs in the given list are a TP, a FP or mm
        B_gt_list is a list of GT-Bounding Boxes in the given frame
        B_pr_list is a list of predicted Bounding Box which should be tested
        
        returns a list of triple which are structured like:
        
        first value:
        0 -> given B_pr is a TP case
        1 -> given B_pr is a FP case
        2 -> given B_pr is a mm (mismatch) case 
        
        second value:
        if the case is TP there is a second return value
        which indicates the index of the GT-Object which 
        matched to the given B_pr        
        if the case is not TP the second value is 'None'
        
        third value:
        if the case is TP there is a third return value
        which is the iou value between the B_pr of this line
        and the matched B_gt
        if the case is not TP the thrid value is 0
    '''
    iou_mat = genIouMat(B_gt_list, B_pr_list)
    
    res_list = []
    
    for row in iou_mat: # loop through all B_pr
        max_iou = np.max(row) # get the max iou for this B_pr
        
        #### FP Determination ####
        if max_iou < threshold: # FP case
            res_list.append((1, None, 0))
            continue
        
        # max_iou >= threshold:
        row_idx = np.where(iou_mat == row)[0][0]
        
        col_idx = np.where(row == max_iou)[0][0]
        
        # get a list of all indices for this B_gt that have a greater iou than the found max_iou            
        ious_greater_idx_list = np.where(iou_mat[:, col_idx] > max_iou)
            
        for idx in ious_greater:                
            # new_row_idx = np.where(iou_mat[:, col_idx] == max_in_col)[0][0] # get the index of this B_pr
            
            if getClass(B_pr_list[idx]) == getClass(B_gt_list[col_idx]): 
                # there is another B_pr which matches better to the found B_gt,
                # the current B_gt is a FP                   
                res_list.append((1, None, 0)) # append the index of the B_pr to the list
                break # get out of the inner for loop
        
            
        #### Mismatch Determination ####       
        # there is no other B_pr for the found B_gt:
        # compare the classes:
        if getClass(B_pr_list[row_idx]) != getClass(B_gt_list[col_idx]): # mismatch found
            # TODO?:
            # should here be tested, if there is another iou value for this B_pr 
            # which is also greater than threshold? -> B_pr can be a TP anyway
            # get all iou values between threshold and max_iou -> other possible matches
            # check every value starting with the highest if it is a tp 
            # if this is the case the given B_pr is a tp, but matches with another B_gt
            other_pos_matches_idx_list = np.where(row >= threshold and row < iou_max)
            other_pos_matches_idx_list.sort()
            # for idx in other_pos_matches_idx_list:
                
                
            res_list.append((2, None, 0))
            continue
    
        # otherwise the B_pr is a TP:
        res_list.append((0, col_idx, max_iou))
        
    return res_list
    
    
def isFN(B_gt_list, B_pr_list, threshold):
    '''
        determines if there is a B_pr for each B_gt in B_gt_list
        B_gt_list is a list of GT-Bounding Boxes in the given frame
        B_pr_list is a list predicted Bounding Boxes which should be tested
        
        returns a list of bool values which determine whether 
        the B_gt has a matching B_pr
        
        return values:
        true -> no matching B_pr to this B_gt, a FN case is detected
        false -> no FN case
    '''
    iou_mat = genIouMat(B_gt_list, B_pr_list)
    res_list = []
    
    for col in iou_mat[:, 0]: # loop through the columns, means the B_gt objects
        
        max_iou = np.max(col)
        
        if max_iou < threshold:
            res_list.append(True)
        
        else:
            res_list.append(False)
    
    return res_list   


def genIouMat(B_gt_list, B_pr_list):
    '''
        generates a matrix of all iou values
        rows: Cam-Objects (B_pr)
        columns: GT-Objects (B_gt)
    '''
    row_count = len(B_pr_list)
    col_count = len(B_gt_list)
    
    iou_mat = np.zeros((row_count, col_count))
    
    cur_row_idx = 0
    
    # fill iou matrix
    for row_idx in range(row_count):
        for col_idx in range(col_count):
            iou_val = iou(B_gt_list[col_idx], B_pr_list[row_idx])
            iou_mat[row_idx][col_idx] = iou_val

    return iou_mat


def getTPs(B_gt_list, B_pr_list, threshold):
    '''
        returns a list of tupels with the indices of matching objects
        like (gt_index, pr_index)
        the indices refer to the indices in the passed lists
    '''
    tp_list = []
    
    iou_mat = genIouMat(B_gt_list, B_pr_list)
    
    for row in iou_mat: # loop through the rows of the matrix, means through B_prs
        max_iou = np.max(row) # get the maximum iou of all B_gts for this B_pr
        
        if max_iou < threshold: # FP case 
            continue
        
        # get the indices:
        row_idx = iou_mat.index(row)
        col_idx = row.index(max_iou)
            
        # check the classes
        if getClass(B_pr_list[row_idx]) != getClass(B_gt_list[col_idx]): # missmatch found
            # TODO:
            # should here be tested, if there is another iou value for this B_pr 
            # which is also greater than threshold? -> B_pr can be a TP anyway
            continue
        
        # if there is another match for this B_gt
        # this B_pr would be a FP
        max_in_col = np.max(iou_mat[:, col_idx])
        
        if max_in_col != max_iou: # there is a better match for this B_gt
            # TODO:
            # should here be tested, if the greater iou value of this B_gt is a mismatch?
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
        if getClass(B_pr_list[row_idx]) == getClass(B_gt_list[col_idx]): # rather a TP case
            continue
        
        # if there is another match for this B_gt
        # this B_pr would be a FP
        max_in_col = np.max(iou_mat[:, col_idx])
        
        if max_in_col != max_iou: 
            continue
        
        # TODO:
        # should here be tested, if there is another iou value for this B_pr 
        # which is also greater than threshold? -> B_pr can be a TP anyway

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
                
        max_iou = np.max(row) # get the maximum iou of all B_gts for this B_pr
        row_idx = iou_mat.index(row)
        col_idx = row.index(max_iou)
        
        ## case 1
        if max_iou < threshold: 
            # FP found
            fp_list.append(row_idx) # append the index of the B_pr to the list
            continue
        
        ## case 2
        # max_iou > threshold
        if max_iou < np.max(iou_mat[:, col_idx]): # there is a greater iou value for this B_gt
            # TODO:
            # should here be tested, if the greater iou value of this B_gt is a mismatch?
            fp_list.append(row_idx) # append the index of the B_pr to the list
    
    return fp_list
