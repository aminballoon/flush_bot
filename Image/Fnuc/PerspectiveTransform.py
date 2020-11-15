# import the necessary packages
import numpy as np
import cv2
import time
cap = cv2.VideoCapture(0)
Pt = [] 
count = 0
width, height = 400, 400

def click_event(event, x, y, flags, param):
    
    global count, Pt
    if event == cv2.EVENT_LBUTTONDOWN and count <= 4 :
        Pt.append([x,y])
        cv2.circle(frame,(x,y),15,(0,0,255),cv2.FILLED)
        count += 1

    
#calling the mouse click event

while(True):
    ret, frame = cap.read()
    for i in Pt:
        cv2.circle(frame,(i[0],i[1]),15,(0,0,255),cv2.FILLED)
    cv2.imshow('frame',frame)
    cv2.setMouseCallback("frame", click_event)
    if count >= 4:
        pts1 = np.float32(Pt)
        pts2 = np.float32([[0,0],[width,0],[0,height],[width,height]])
        matrix = cv2.getPerspectiveTransform(pts1,pts2)
        frameOutput = cv2.warpPerspective(frame,matrix,(width,height))
        cv2.imshow("Set",frameOutput)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cv2.destroyAllWindows()
