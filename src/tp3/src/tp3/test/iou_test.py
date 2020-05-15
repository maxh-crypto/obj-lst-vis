import sys 
sys.path.append('../')

import DataEvaluation_module as eval

B_gt_1 = eval.BoundingBox2D(0.0, 0.0, 4.0, 2.0, 0.0)
B_pr_1 = eval.BoundingBox2D(1.0, 1.0, 4.0, 2.0, 0.0)

B_gt_2 = eval.BoundingBox2D(0.0, 0.0, 4.0, 2.0, 0.0)
B_pr_2 = eval.BoundingBox2D(1.0, 1.0, 4.0, 2.0, 1.0)

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




