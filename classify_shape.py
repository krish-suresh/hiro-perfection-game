import cv2
import numpy as np


def mask_largest_contour(input):
    input_blur = cv2.GaussianBlur(input,(5,5),cv2.BORDER_DEFAULT)
    hsv = cv2.cvtColor(input_blur, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([15,0,0])
    upper_yellow = np.array([36, 255, 255])
    mask_all = cv2.inRange(hsv, lower_yellow, upper_yellow)
    ret,thresh = cv2.threshold(mask_all, 40, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    mask = np.zeros(input.shape[0:2], np.uint8)
    if len(contours) != 0:
        c = max(contours, key = cv2.contourArea)
        cv2.fillPoly(mask, pts =[c], color=255)
    return mask


input_0 = cv2.imread('shapes/piece_0.png')
input_1 = cv2.imread('shapes/piece_0.png')
input_0_mask = mask_largest_contour(input_0)
input_1_mask = mask_largest_contour(input_1)
intersection = cv2.bitwise_and(input_0_mask,input_1_mask)
union = cv2.bitwise_or(input_0_mask,input_1_mask)
print(cv2.countNonZero(intersection))
print(cv2.countNonZero(union))
print(cv2.countNonZero(intersection)/cv2.countNonZero(union))
# cv2.imshow("input_mask_0",input_0_mask)
# cv2.imshow("input_mask_1",input_1_mask)
cv2.imshow("intersection",intersection)
# cv2.waitKey(0)
cv2.imshow("union",union)
cv2.waitKey(0)
cv2.destroyAllWindows() 
# Find contour of input image
# Loop for 25 images
#   find centroid for both input and test_shape
#   shift mask so centroid of input and test_shape is at center of image
#   Loop for rotations
#       rotate input mask by rotation val
#       find intersection between input and test_shape - size of bitwise and
#       find union between input and test_shape - size of bitwise or
#       if iou is greater than last max store save rotation and shape as best fit