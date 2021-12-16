import classify_shape
import cv2 
import numpy as np

cam = cv2.VideoCapture(1)
cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
focus = 100
cam.set(28, focus) 
# frame = cv2.imread('game_pieces.jpg')
prev_shape = 1
lower_yellow_cam = np.array([15,50,50])
upper_yellow_cam = np.array([50, 255, 255])

mouseX,mouseY = 0,0
def update_mouse(event,x,y,flags,param):
    global mouseX,mouseY
    if event == cv2.EVENT_LBUTTONDBLCLK:
        mouseX,mouseY = x,y
cv2.namedWindow('frame')
cv2.setMouseCallback('frame',update_mouse)

while(1):
    ret, frame = cam.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if mouseX:
        print(hsv[mouseY, mouseX])
        mouseX = None
    input_mask = classify_shape.mask_largest_contour(frame,lower_yellow_cam, upper_yellow_cam)
    predicted_shape = classify_shape.classify(frame, range(25))
    if predicted_shape[0] != prev_shape:
        predicted_shape_picture = cv2.imread('shapes_update/piece_{}.png'.format(predicted_shape[0]))
        prev_shape = predicted_shape[0]
        print("Piece {} rotated by {} degrees".format(predicted_shape[0],predicted_shape[1]))
    cv2.imshow('frame',frame)
    cv2.imshow('mask', input_mask)
    cv2.imshow('shape', predicted_shape_picture)
    k = cv2.waitKey(1)
    if k%256 == 27:
        break
cam.release()
cv2.destroyAllWindows() 