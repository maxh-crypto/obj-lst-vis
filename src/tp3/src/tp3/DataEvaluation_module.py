'''
    this module can compare to bag files and computes evaluation values, 
    such as IoU, TP, FP, FN, Precision, Recall, FPPI, MOTA, MOTP
'''

import shapely.geometry
import shapely.affinity

THRESHOLD = 0.5


def intersection(B_gt, B_pr):
    '''
        calculates the intersection of the given bounding boxes 
        B_gt (Ground Truth) and B_pr (Predicted)
    '''
    intersection = B_gt.get_contour().intersection(B_pr.get_contour())
    
    return intersection.area


def union(B_gt, B_pr):
    '''
        calculates the union of the given bounding boxes 
        B_gt (Ground Truth) and B_pr (Predicted)
    '''
    return B_gt.area() + B_pr.area() - intersection(B_gt, B_pr)


def iou(B_gt, B_pr):
    '''
        calculates the intersection over union of the given bounding boxes
        B_gt (Ground Truth) and B_pr (Predicted)
    '''
    return intersection(B_gt, B_pr) / union(B_gt, B_pr)


class BoundingBox2D():
    pos_x = 0.0
    pos_y = 0.0
    length = 0.0
    width = 0.0
    yaw = 0.0
    
    def __init__(self, x, y, l, w, yaw):
        self.pos_x = x
        self.pos_y = y
        self.length = l
        self.width = w
        self.yaw = yaw
        
    def get_contour(self):
        l = self.length
        w = self.width
        c = shapely.geometry.box(-l / 2.0, -w / 2.0, l / 2.0, w / 2.0)
        rc = shapely.affinity.rotate(c, self.yaw)
        return shapely.affinity.translate(rc, self.pos_x, self.pos_y)
    
    def area(self):
        return self.get_contour().area

    def intersection(self, other):
        intersection = self.get_contour().intersection(other.get_contour())
        return intersection.area 
    
    def union(self, other):
        return self.area + other.area - self.intersection(other)
    
    def iou(self, other):
        return self.intersection(other) / self.union(other)
    
    
