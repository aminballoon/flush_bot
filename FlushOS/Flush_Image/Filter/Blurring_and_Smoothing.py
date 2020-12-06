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


top_row =    [32, 31, 30, 29, 28, 27, 26, 25, 24]
left_row =   [32, 1, 2, 3, 4, 5, 6, 7, 8]
right_row =   [24, 23, 22, 21, 20, 19, 18, 17, 16]
bottom_row = [8, 9, 10, 11, 12, 13, 14, 15, 16]

print(top_row[::-1])
print(left_row[::-1])
print(right_row[::-1])
print(bottom_row[::-1])