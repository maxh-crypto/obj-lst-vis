import sys 
sys.path.append('../')

import DataEvaluation_module as eval

class RosObject():
    def __init__(self, x, y, l, w, yaw=0.0, cls=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        self.geometric = Geometric(x, y, yaw)
        self.dimension = Dimension(l, w)
        self.classification = Classification(cls)

class Geometric():
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        
class Dimension():
    def __init__(self, l, w):
        self.length = l
        self.width = w        
    
class Classification():
    def __init__(self, prop_values):
        self.car = prop_values[0]
        self.truck = prop_values[1]
        self.motorcycle = prop_values[2]
        self.bicycle = prop_values[3]
        self.pedestrian = prop_values[4]
        self.stacionary = prop_values[5]
        self.other = prop_values[6]

B_gt_1 = RosObject(0.0, 0.0, 4.0, 2.0, 0.0)
B_pr_1 = RosObject(1.0, 1.0, 4.0, 2.0, 0.0)

B_gt_2 = RosObject(8.0, 10.0, 4.0, 2.0, 0.0, [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
B_pr_2 = RosObject(8.0, 10.0, 4.0, 2.0, 1.0)

B_gt_3 = RosObject(16.0, 25.0, 4.0, 2.0, 0.0)
B_pr_3 = RosObject(4.0, 5.0, 4.0, 2.0, 0.0)


### for simple objects in line
## intersection:
intersection = eval.intersection(B_gt_1, B_pr_1)
assert(intersection == 3.0), "wrong intersection value"
print("calculated intersection " + str(intersection))

## union:
union = eval.union(B_gt_1, B_pr_1)
assert(union == 13.0), "wrong union value"
print("calculated union " + str(union))

## iou:
iou = eval.iou(B_gt_1, B_pr_1)
assert(iou == (3.0 / 13.0)), "wrong iou value"
print("calculated iou " + str(iou))
    
    
### rotated objects
print("***** test rotated object *****")
## intersection:
intersection = eval.intersection(B_gt_2, B_pr_2)
# assert(intersection == 3.0), "wrong intersection value"
print("calculated intersection " + str(intersection))

## union:
union = eval.union(B_gt_2, B_pr_2)
# assert(union == 13.0), "wrong union value"
print("calculated union " + str(union))

## iou:
iou = eval.iou(B_gt_2, B_pr_2)
# assert(iou == (3.0 / 13.0)), "wrong iou value"
print("calculated iou " + str(iou))


print("***** test for TP/FP/mm *****")
case, index = eval.det_TP_FP_mm([B_gt_1, B_gt_2, B_gt_3], B_pr_2, 0.5)
if case == 0: # TP
    print("TP case found. B_pr matched with B_gt_" + str(index + 1))
elif case == 1: # FP
    print("FP case detected.")
else:
    print("Mismatch detected with B_gt_" + str(index + 1))

