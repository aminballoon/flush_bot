import cv2
import numpy as np
from math import sqrt,pi,atan2,degrees,cos,sin
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



# for q in range(len(list_approx)-1):
#             Delta_y = list_approx[q][0] - list_approx[q+1][0]
#             Delta_x = list_approx[q][1] - list_approx[q+1][1]
#             if int(sqrt((Delta_y * Delta_y) + (Delta_x * Delta_x))) >= 20:
#                 print(str(list_approx[q]) + " " + str(list_approx[q+1]))

# p = [[335, 204], [297, 197], [338, 212], [334, 341], [133, 346], [335, 341]]
# while(shit != 0):
#     for q in range(len(list_approx)-1):
#         Delta_y = list_approx[q][0] - list_approx[q+1][0]
#         Delta_x = list_approx[q][1] - list_approx[q+1][1]
#         if int(sqrt((Delta_y * Delta_y) + (Delta_x * Delta_x))) >= 20:



# p = [[285, 145], [191, 308], [151, 324], [191, 308]]
# ans = []
# for i in p:
    # print(i)
    # if ans == []:
        # ans.append(i)
    # else:
        # if ans[-1]
            # print( abs(i[0]-j[0]) , abs(i[1]-j[1]))  
            # ans.append(i)
    # for j in ans:
    #     print(j,i)
    #     # print(j[0] - i[0])
    #     # print(i)
    #     if (j[0] - i[0] <= 10) and (j[1] - i[1] <= 10) :
    #         print(j[0] - i[0])
    #         ans.append(i)

# for i in range(len(p)):
#     print(p[i])
#     if ans == []:
# #         ans.append(i)
# print(ans)
        
    
    
# Kuy = [ (311 , 45),
# (151 , 44),
# (116 , 51),
# (50 , 121),
# (42 , 148),
# (41 , 302),
# (123 , 301),
# (294 , 297),
# (298 , 174),
# (245 , 171),
# (181 , 170) ]

# Kuy = [(311 , 45),
# (294 , 297),
# (116 , 51),
# (151 , 44),
# (41 , 302),
# (50 , 121),
# (42 , 148),
# (298 , 174),
# (123 , 301),
# (245 , 171),
# (181 , 170)]

# Ans = []
# Ans.append(Kuy[0])

# for i in range(len())

# Kuy = [(199,199),(198,198),(201,201)]
# re = []
# ans = []

# def Sam_OOk(List_Point,thres):
#     for i in range(0,len(List_Point)):
#         for j in range(i+1,len(List_Point)):
#             if (abs(List_Point[i][0] - List_Point[j][0]) < thres) or (abs(List_Point[i][1] - List_Point[j][1]) < thres):
#                 re.append(j)
#     for i in range(0,len(List_Point)):
#         if i not in re:
#             ans.append(List_Point[i])
#     return ans

# print(Sam_OOk(Kuy,50))


# top_row =    [32, 31, 30, 29, 28, 27, 26, 25, 24]
# left_row =   [32, 1, 2, 3, 4, 5, 6, 7, 8]
# right_row =   [24, 23, 22, 21, 20, 19, 18, 17, 16]
# bottom_row = [8, 9, 10, 11, 12, 13, 14, 15, 16]

# print(top_row[::-1])
# print(left_row[::-1])
# print(right_row[::-1])
# print(bottom_row[::-1])


# def Born_To_be_Flushbot(list_cmd):
#     ans = []
#     for i in range(len(list_cmd)-1):
#         if i == 0:
#             ans.append(list_cmd[0] + (0,))
#         elif i == 1:
#             ans.append((0,0,list_cmd[1][2]))
#             ans.append(list_cmd[i])
#         # if i == len(list_cmd)-2:
#         #     ans.append(list_cmd[i])
#         else:
#             ans.append(list_cmd[i])
#         print(i , ans)
#     ans.append(list_cmd[-1]+(list_cmd[-2][2],))
#     return ans

# print(Born_To_be_Flushbot([(330, 72), (230, 77, 270), (196, 92, 270), (113, 240, 170), (67, 306)]))

#List_of_Position = [(330, 72, 0), (0, 0, 270),(230, 77, 270), (196,92, 270),(113, 240, 180), (67, 306, 170)]

# def Targectory_Gen(x1,y1,x2,y2):
#     P1 = (x1,y1)
#     P2 = (x2,y2)
#     delta_x = P2[0]-P1[0]
#     delta_y = P2[1]-P1[1]
#     r = sqrt(pow(delta_x,2)+pow(delta_y,2))
    
#     Theta = atan2(delta_y,delta_x)
#     Theta = int(degrees(Theta))
#     if Theta < 0:
#         Theta += 180 
#     Theta = Theta/2
#     return int(r),int(Theta), int(r*cos(Theta)) , int(r*sin(Theta))

# for Theta in range(-180,0):
#     P = Theta
#     if Theta < 0:
#         Theta += 180 
#     # if Theta > 90:
#     #     Theta = 180 - Theta 
#     print(P,Theta)

# List_of_Position =  [(54, 330+5, 0),(0,0,260),
# (143, 329+5, 250),
# (247, 318+5, 160),
# (331, 213+5, 160),
# (331, 142+5, 260),
# (331, 53+5, 260),
# (260, 55+5, 260),
# (51, 50+5, 160),
# (59, 190+5, 160),
# (105, 193+5, 160),
# (180, 191+5, 160)]

# def Theta_isas(List_of_Position):
#     ans = []
#     _,Theta,_,_ = Targectory_Gen(List_of_Position[0][1],List_of_Position[0][0],List_of_Position[2][1],List_of_Position[2][0])
#     ans.append(List_of_Position[0]+(Theta,))
#     ans.append(List_of_Position[1]+(Theta,))
#     for i in range(2,len(List_of_Position)-1):
#         print(i,i+1)
#         _,Theta,_,_ = Targectory_Gen(List_of_Position[i][1],List_of_Position[i][0],List_of_Position[i+1][1],List_of_Position[i+1][0])
#         ans.append(List_of_Position[i]+(Theta,))
#     _,Theta,_,_ = Targectory_Gen(List_of_Position[-2][1],List_of_Position[-2][0],List_of_Position[-1][1],List_of_Position[-1][0])
#     ans.append(List_of_Position[-1]+(Theta,))
#     return ans
# print(Theta_isas(List_of_Position))

# c = cv2.imread(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_main\Flush_Image\Capture_image\Image\Image_Anti_Obstacle2020_12_06_00_46_00_601762.png',0)
# cv2.imwrite(r"C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\222.png",c)
