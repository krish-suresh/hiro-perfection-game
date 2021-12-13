import classify_shape
import cv2
import numpy as np

cam = cv2.VideoCapture(0, cv2.CAP_DSHOW)
#  Hue Sat Value
lower_yellow = np.array([15,50,50])
upper_yellow = np.array([50, 255, 255])
mouseX,mouseY = 0,0

def update_mouse(event,x,y,flags,param):
    global mouseX,mouseY
    if event == cv2.EVENT_LBUTTONDBLCLK:
        mouseX,mouseY = x,y
cv2.namedWindow('frame')
cv2.setMouseCallback('frame',update_mouse)
while True:
    ret, frame = cam.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if mouseX:
        print(hsv[mouseY, mouseX])
        mouseX = None
    cv2.imshow('frame', frame)
    pic = classify_shape.mask_largest_contour(frame, lower_yellow, upper_yellow)
    cv2.imshow('pic', pic)
    k = cv2.waitKey(1)
    if k%256 == 27:
        break
cam.release()
cv2.destroyAllWindows() 