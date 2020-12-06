
import cv2
import numpy as np

#This will display all the available mouse click events  
events = [i for i in dir(cv2) if 'EVENT' in i]
# print(events)

#This variable we use to store the pixel location
refPt = []

Point_Per = []
#click event function
def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(x,",",y)
        refPt.append([x,y])
        font = cv2.FONT_HERSHEY_SIMPLEX
        strXY = str(x)+", "+str(y)
        Point_Per.append(strXY)
        cv2.putText(img, strXY, (x,y), font, 0.5, (255,255,0), 2)
        cv2.imshow("image", img)


#Here, you need to change the image name and it's path according to your directory
img = cv2.imread(r"C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_main\Flush_Image\Capture_image\Image\Image_Anti_Obstacle2020_12_07_00_38_14_975985.png")
cv2.imshow("image", img)

#calling the mouse click event
cv2.setMouseCallback("image", click_event)

cv2.waitKey(0)
cv2.destroyAllWindows()