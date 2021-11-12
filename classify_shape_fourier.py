import cv2
import numpy as np
import matplotlib.pyplot as plt

def get_largest_contour(input):
    input_blur = cv2.GaussianBlur(input,(5,5),cv2.BORDER_DEFAULT)
    hsv = cv2.cvtColor(input_blur, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([0,50,50])
    upper_yellow = np.array([50, 255, 255])
    mask_all = cv2.inRange(hsv, lower_yellow, upper_yellow)
    ret,thresh = cv2.threshold(mask_all, 40, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    mask = np.zeros(input.shape[0:2], np.uint8)
    if len(contours) != 0:
        c = max(contours, key = cv2.contourArea)
        cv2.fillPoly(mask, pts =[c], color=255)
    return mask, c

input = cv2.imread('shapes/piece_13.png')
input_mask,contour = get_largest_contour(input)
print(contour.squeeze().shape)
M = cv2.moments(contour)
cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])
input = cv2.circle(input, (cx,cy), radius=4, color=(0, 0, 255), thickness=-1)
dists = []
for point in contour.squeeze():
    input = cv2.circle(input, point, radius=0, color=(0, 0, 255), thickness=-1)
    dists.append(cv2.norm(point-(cx,cy)))

input = cv2.imread('piece_13_rot.png')
input_mask,contour = get_largest_contour(input)
print(contour.squeeze().shape)
M = cv2.moments(contour)
cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])
input = cv2.circle(input, (cx,cy), radius=4, color=(0, 0, 255), thickness=-1)
dists_rot = []
for point in contour.squeeze():
    input = cv2.circle(input, point, radius=0, color=(0, 0, 255), thickness=-1)
    dists_rot.append(cv2.norm(point-(cx,cy)))
plt.plot(dists)
plt.plot(dists_rot)
plt.show()
# cv2.imshow('input',input)
# cv2.imshow('input_rot',input_mask)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
# quit()


# Find contour of input image
# Loop for 25 images
#   find centroid for both input and test_shape
#   shift mask so centroid of input and test_shape is at center of image
#   Loop for rotations
#       rotate input mask by rotation val
#       find intersection between input and test_shape - size of bitwise and
#       find union between input and test_shape - size of bitwise or
#       if iou is greater than last max store save rotation and shape as best fit