import cv2
import numpy as np
# List_of_Position = [(200,200),
# (257, 172),
# (295, 178),
# (294, 299),
# (113, 303)]

# blacked = np.zeros((400,400 , 1), np.uint8)
# for i in range(len(List_of_Position)-1):
#     cv2.line(blacked,List_of_Position[i],List_of_Position[i+1],(255,255,255))

# cv2.imshow(";lll",blacked)
# cv2.waitKey(0)

# ppp = cv2.imread(r'C:\Users\aminb\Documents\GitHub\flush_bot\PATH_Image.png')
# vv = cv2.resize(ppp,(400,400))

# cv2.imshow(";llasdasdl",ppp)
# cv2.imshow(";lll",vv)
# cv2.waitKey(0)



for q in range(len(list_approx)-1):
            Delta_y = list_approx[q][0] - list_approx[q+1][0]
            Delta_x = list_approx[q][1] - list_approx[q+1][1]
            if int(sqrt((Delta_y * Delta_y) + (Delta_x * Delta_x))) >= 20:
                print(str(list_approx[q]) + " " + str(list_approx[q+1]))

p = [[335, 204], [297, 197], [338, 212], [334, 341], [133, 346], [335, 341]]
while(shit != 0):
    for q in range(len(list_approx)-1):
        Delta_y = list_approx[q][0] - list_approx[q+1][0]
        Delta_x = list_approx[q][1] - list_approx[q+1][1]
        if int(sqrt((Delta_y * Delta_y) + (Delta_x * Delta_x))) >= 20: