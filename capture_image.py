import cv2 
import numpy as np
import time
cam = cv2.VideoCapture(1)
cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
time.sleep(1)
focus = 20
cam.set(28, focus) 
shape_number = 0

while(1):
    _, frame = cam.read()
    cv2.imshow("frame", frame)
    k = cv2.waitKey(1)
    if k%256 == 97:
        cv2.imwrite("shapes_update/piece_{}.png".format(shape_number), frame)
        shape_number+=1
        print(shape_number)
    if k%256== 27:
        break
cam.release()
cv2.destroyAllWindows()