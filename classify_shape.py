import cv2
import numpy as np


def mask_largest_contour(input, rotation=0):
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
        if rotation != 0:
            c = rotate_contour(c,rotation)
        x,y,w,h = cv2.boundingRect(c)
        square_size = max(w,h)
        mask = np.zeros((square_size,square_size), np.uint8)
        cx = int(w/2)+x
        cy = int(h/2)+y
        center = int(square_size/2)
        cv2.fillPoly(mask, pts =[c], color=255,offset=(-cx+center,-cy+center))
    return mask

def intersection_over_union(input, test, input_rotation):
    input_mask = mask_largest_contour(input, rotation=input_rotation)
    test_mask = mask_largest_contour(test)
    input_mask = cv2.resize(input_mask, test_mask.shape, interpolation = cv2.INTER_AREA)
    intersection = cv2.bitwise_and(input_mask,test_mask)
    union = cv2.bitwise_or(input_mask,test_mask)
    return cv2.countNonZero(intersection)/cv2.countNonZero(union)

def cart2pol(x, y):
    theta = np.arctan2(y, x)
    rho = np.hypot(x, y)
    return theta, rho

def pol2cart(theta, rho):
    x = rho * np.cos(theta)
    y = rho * np.sin(theta)
    return x, y

def rotate_contour(cnt, angle):
    M = cv2.moments(cnt)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    cnt_norm = cnt - [cx, cy]
    coordinates = cnt_norm[:, 0, :]
    xs, ys = coordinates[:, 0], coordinates[:, 1]
    thetas, rhos = cart2pol(xs, ys)
    thetas = np.rad2deg(thetas)
    thetas = (thetas + angle) % 360
    thetas = np.deg2rad(thetas)
    xs, ys = pol2cart(thetas, rhos)
    cnt_norm[:, 0, 0] = xs
    cnt_norm[:, 0, 1] = ys
    cnt_rotated = cnt_norm + [cx, cy]
    cnt_rotated = cnt_rotated.astype(np.int32)
    return cnt_rotated
# input = cv2.imread('shapes/piece_0.png')
# input_rot = mask_largest_contour(input, rotation=30)
# cv2.imshow('input',input)
# cv2.imshow('input_rot',input_rot)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
# quit()
input = cv2.imread('piece_13_rot.png')
print(input.shape)
num_test_img = 25
degree_increment = 10
best_fit_test = (0,0) # (shape_num, deg)
max_iou = 0
for i in range(num_test_img):
    test_image = cv2.imread(f'shapes/piece_{i}.png')
    # TODO centroid centering
    for deg in np.linspace(0,270, degree_increment):
        iou = intersection_over_union(input, test_image, deg)
        if  iou > max_iou:
            best_fit_test = (i,deg)
            max_iou = iou
print(best_fit_test)
# Find contour of input image
# Loop for 25 images
#   find centroid for both input and test_shape
#   shift mask so centroid of input and test_shape is at center of image
#   Loop for rotations
#       rotate input mask by rotation val
#       find intersection between input and test_shape - size of bitwise and
#       find union between input and test_shape - size of bitwise or
#       if iou is greater than last max store save rotation and shape as best fit