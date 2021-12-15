import classify_shape
import cv2 
import numpy as np
from collections import deque, Counter

cam = cv2.VideoCapture(1)
cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
focus = 100
cam.set(28, focus) 
# frame = cv2.imread('game_pieces.jpg')
prev_shape = 1
lower_yellow_cam = np.array([15,150,50])
upper_yellow_cam = np.array([50, 255, 200])
predicted_shapes = []

for i in range(10):
    ret, frame = cam.read()
    input_mask = classify_shape.mask_largest_contour(frame,lower_yellow_cam, upper_yellow_cam)
    predicted_shape = classify_shape.classify(frame, range(25))
    if predicted_shape[0] != prev_shape:
        predicted_shape_picture = cv2.imread('shapes/piece_{}.png'.format(predicted_shape[0]))
        prev_shape = predicted_shape[0]
        print("Piece {} rotated by {} degrees".format(predicted_shape[0],predicted_shape[1]))
    predicted_shapes.append(predicted_shape)
    cv2.imshow('frame',frame)
    cv2.imshow('mask', input_mask)
    cv2.imshow('shape', predicted_shape_picture)
    # k = cv2.waitKey(1)
    # if k%256 == 27:
    #     break
print(predicted_shapes)
occurence_count = Counter(predicted_shapes)
predicted_shape = occurence_count.most_common(1)[0][0]
cam.release()
cv2.destroyAllWindows() 
