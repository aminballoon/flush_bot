import numpy as np
import cv2
import cv2.aruco as aruco
cap = cv2.VideoCapture(1+cv2.CAP_DSHOW)
point_per = []
ids = []
while(True):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=arucoParameters)
    frame = aruco.drawDetectedMarkers(frame, corners)
    if ids != []:
        print(ids)
        index_id = ids.tolist().index([2]) # 5
        point = tuple(corners[index_id].astype(int).tolist()[0][2])
        point_per.append(point)
        # cv2.circle(frame,point,5,(0,222,222),-1)
        # elif int(i) == 7:     
        index_id = ids.tolist().index([0]) #7
        point = tuple(corners[index_id].astype(int).tolist()[0][3])
        point_per.append(point)
        # cv2.circle(frame,point,5,(0,222,222),-1)
        # elif int(i) == 1:
        index_id = ids.tolist().index([3]) #1
        point = tuple(corners[index_id].astype(int).tolist()[0][1])
        point_per.append(point)
        # cv2.circle(frame,point,5,(0,222,222),-1)
        # elif int(i) == 3:
        index_id = ids.tolist().index([1]) #3
        point = tuple(corners[index_id].astype(int).tolist()[0][0])
        point_per.append(point)
        # cv2.circle(frame,point,5,(0,222,222),-1)
        

        matrix = cv2.getPerspectiveTransform(np.float32(point_per),np.float32([[0,0],[400,0],[0,400],[400,400]]))
        frameOutput = cv2.warpPerspective(frame,matrix,(400,400))
        cv2.imshow('Display', frameOutput)
    cv2.imshow('Display', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()

# frame = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\Moduel_image\per.jpg')
# gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
# arucoParameters = aruco.DetectorParameters_create()
# corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters,)

# frame = aruco.drawDetectedMarkers(frame, corners)

# for i in range(len(corners)):
#     for j in corners[i].astype(int).tolist()[0]:
#         print(j)
point_per = []

# if int(i) == 5:
index_id = ids.tolist().index([2]) # 5
point = tuple(corners[index_id].astype(int).tolist()[0][2])
point_per.append(point)
# cv2.circle(frame,point,5,(0,222,222),-1)
# elif int(i) == 7:     
index_id = ids.tolist().index([0]) #7
point = tuple(corners[index_id].astype(int).tolist()[0][3])
point_per.append(point)
# cv2.circle(frame,point,5,(0,222,222),-1)
# elif int(i) == 1:
index_id = ids.tolist().index([3]) #1
point = tuple(corners[index_id].astype(int).tolist()[0][1])
point_per.append(point)
# cv2.circle(frame,point,5,(0,222,222),-1)
# elif int(i) == 3:
index_id = ids.tolist().index([1]) #3
point = tuple(corners[index_id].astype(int).tolist()[0][0])
point_per.append(point)
# cv2.circle(frame,point,5,(0,222,222),-1)
    

matrix = cv2.getPerspectiveTransform(np.float32(point_per),np.float32([[0,0],[400,0],[0,400],[400,400]]))
frameOutput = cv2.warpPerspective(frame,matrix,(400,400))

cv2.imshow('Displays', frame)
cv2.imshow('Display', frameOutput)
cv2.waitKey(0)



