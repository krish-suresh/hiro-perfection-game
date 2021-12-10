import classify_shape

import cv2 
import numpy as np

cam = cv2.VideoCapture(0)
# frame = cv2.imread('game_pieces.jpg')
prev_shape = 1
while(1):
    ret, frame = cam.read()
    predicted_shape = classify_shape.classify(frame)
    if predicted_shape[0] != prev_shape:
        predicted_shape_picture = cv2.imread(f'shapes/piece_{predicted_shape[0]}.png')
        prev_shape = predicted_shape[0]

    print(f"Piece {predicted_shape[0]} rotated by {predicted_shape[1]} degrees")
    cv2.imshow('frame',frame)
    cv2.imshow('shape', predicted_shape_picture)
    k = cv2.waitKey(1)
    if k%256 == 27:
        break
cam.release()
cv2.destroyAllWindows() 