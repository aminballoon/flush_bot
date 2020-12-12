import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl

#detect ARUCO --> corners, ids of all ARUCO in an image
def detect_aruco(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    return corners, ids

#take ARUCO corner as input, Compute center of that ARUCO and return as Output
def compute_aruco_center(corner):
    c = corner[0]
    center = [int(c[:, 0].mean()), int(c[:, 1].mean())]
    return center

#compute point for perspective transfrom
def get_all_corner(corners, ids, list_of_corner_id):
    corner_point = []
    for c in range(len(corners)):
        if ids[c] in list_of_corner_id:
            center = compute_aruco_center(corners[c])
            corner_point.append(center)
    return corner_point

# c,d = (detect_aruco(cv2.imread(r"C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_Communication\Rail_Photo\Photo0.jpg")))

frame = cv2.imread(r"C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_Communication\Rail_Photo\Photo17.jpg")
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

while(True):
    # Capture frame-by-frame


    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    res = cv2.aruco.detectMarkers(gray,dictionary)
#   print(res[0],res[1],len(res[2]))

    if len(res[0]) > 0:
        cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])
    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()