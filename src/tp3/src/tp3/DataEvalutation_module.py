'''
    this module can compare to bag files and computes evaluation values, 
    such as IoU, TP, FP, FN, Precision, Recall, FPPI, MOTA, MOTP
'''

import shapely.geometry
import shapely.affinity

def intersection(B_gt, B_pr):
    '''
        calculates the intersection of the given bounding boxes 
        B_gt (Ground Truth) and B_pr (Predicted)
    '''
    pass

def union(B_gt, B_pr):
    '''
        calculates the union of the given bounding boxes 
        B_gt (Ground Truth) and B_pr (Predicted)
    '''

class BoundingBox2D():
    pos_x = 0.0
    pos_y = 0.0
    length = 0.0
    height = 0.0
    yaw = 0.0
    
    def __init__(self, x, y, l, h, yaw):
        self.pos_x = x
        self.pos_y = y
        self.length = l
        self.height = h
        self.yaw = yaw
        
    def get_contour(self):
        w = self.w
        h = self.h
        c = shapely.geometry.box(-w/2.0, -h/2.0, w/2.0, h/2.0)
        rc = shapely.affinity.rotate(c, self.angle)
        return shapely.affinity.translate(rc, self.cx, self.cy)

    def intersection(self, other):
        return self.get_contour().intersection(other.get_contour()) 

    
    