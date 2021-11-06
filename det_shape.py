import cv2 
import numpy as np

cam = cv2.VideoCapture(0)
# frame = cv2.imread('game_pieces.jpg')
while(1):
    ret, frame = cam.read()
    output = frame.copy()
    frame_blur = cv2.GaussianBlur(frame,(5,5),cv2.BORDER_DEFAULT)
    # prob add a blur here  
    if not ret:
        print("failed to grab frame")
        break
    hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)
        
    lower_yellow = np.array([15,0,0])
    upper_yellow = np.array([36, 255, 255])

    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # output = cv2.bitwise_and(frame,frame, mask= mask)
    ret,thresh = cv2.threshold(mask, 40, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) != 0:
        # cv2.drawContours(frame, contours, -1, 255, 3)
        c = max(contours, key = cv2.contourArea)
        cv2.drawContours(output, c, -1, 255, 3)
        # x,y,w,h = cv2.boundingRect(c)
        # cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

    cv2.imshow('frame',frame)
    # cv2.imshow('frame_blur',frame_blur)
    # cv2.imshow('mask',mask)
    cv2.imshow('output',output)
    k = cv2.waitKey(1)
    if k%256 == 27:
        break
cam.release()
cv2.destroyAllWindows() 