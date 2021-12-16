import cv2
import numpy as np


def mask_largest_contour(input, hsv_low, hsv_high, rotation=0):
    input_blur = cv2.GaussianBlur(input,(5,5),cv2.BORDER_DEFAULT)
    hsv = cv2.cvtColor(input_blur, cv2.COLOR_BGR2HSV)
    mask_all = cv2.inRange(hsv, hsv_low, hsv_high)
    ret,thresh = cv2.threshold(mask_all, 40, 255, 0)
    contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[1]
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
    lower_yellow = np.array([0,50,50])
    upper_yellow = np.array([50, 255, 255])
    lower_yellow_cam = np.array([15,100,50])
    upper_yellow_cam = np.array([50, 255, 255])
    input_mask = mask_largest_contour(input,lower_yellow_cam, upper_yellow_cam, rotation=input_rotation)
    test_mask = mask_largest_contour(test,lower_yellow, upper_yellow)
    input_mask = cv2.resize(input_mask, test_mask.shape, interpolation = cv2.INTER_AREA)
    intersection = cv2.bitwise_and(input_mask,test_mask)
    union = cv2.bitwise_or(input_mask,test_mask)
    return float(cv2.countNonZero(intersection))/float(cv2.countNonZero(union)), input_mask, test_mask

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
# lower_yellow = np.array([0,50,50])
# upper_yellow = np.array([50, 255, 255])
# input_rot = mask_largest_contour(input,lower_yellow, upper_yellow)
# cv2.imshow('input',input)
# cv2.imshow('input_rot',input_rot)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
# quit()

def classify(input, img_list):
    degree_increment = 10
    best_fit_test = (0,0) # (shape_num, deg)
    max_iou = 0
    best_mask_input = None
    best_mask_test = None
    for i in img_list:
        test_image = cv2.imread('shapes_update/piece_{}.png'.format(i))
        for deg in np.linspace(0,180, degree_increment):
            iou, mask_input, mask_test = intersection_over_union(input, test_image, deg)
            if  iou > max_iou:
                best_fit_test = (i,deg)
                max_iou = iou
                best_mask_input = mask_input
                best_mask_test = mask_test

    if best_mask_input is not None and best_mask_test is not None:
        cv2.imshow('best_mask_input', best_mask_input)
        cv2.imshow('best_mask_test', best_mask_test)
    return best_fit_test

# input = cv2.imread('piece_13_rot.png')
# print(input.shape)
# print(classify(input))