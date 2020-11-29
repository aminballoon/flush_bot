# import numpy
# import cv2
# from scipy.interpolate import splprep, splev

# img = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\Moduel_image\Filed_img.jpg')
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# blur = cv2.GaussianBlur(gray, (5,5), 0)
# thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
# contours, hierarchy = cv2.findContours(
#             thresh , 
#             cv2.RETR_TREE, 
#             cv2.CHAIN_APPROX_NONE) 

# smoothened = []
# print(len(contours))
# for contour in contours:
#     x,y = contour.T
#     # Convert from numpy arrays to normal arrays
#     x = x.tolist()[0]
#     y = y.tolist()[0]
#     # https://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.interpolate.splprep.html
#     tck, u = splprep([x,y], u=None, s=1.0, per=1)
#     # https://docs.scipy.org/doc/numpy-1.10.1/reference/generated/numpy.linspace.html
#     u_new = numpy.linspace(u.min(), u.max(), 25)
#     # https://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.interpolate.splev.html
#     x_new, y_new = splev(u_new, tck, der=0)
#     # Convert it back to numpy format for opencv to be able to display it
#     res_array = [[[int(i[0]), int(i[1])]] for i in zip(x_new,y_new)]
#     smoothened.append(numpy.asarray(res_array, dtype=numpy.int32))

# # Overlay the smoothed contours on the original image
# # cv2.drawContours(img, smoothened, -1, (0,0,255), 2)
# cv2.drawContours(img, contours, -1, (0,0,255), 2)
# cv2.imshow("sss",img)
# cv2.waitKey(0)
# my_list = [1,1,1,2,3,4,5,6,213,12,3,123,12,31,231,23]
# # print(ooo.pop())
# for _ in range(int(len(my_list)/2)):
#     print(my_list.pop())
#     # ooo = ooo.pop()



# def top_to_bot(list_is):
#     ans = list_is[1:len(list_is)]
#     ans.append(list_is[0])
#     return ans
# print(top_to_bot([1,2,3,4,5]))

# [[301, 497], [276, 497],
# [302, 651], [278, 674],
# [740, 652], [741, 673] ]
# x = [[301, 497], [276, 497],[740, 652],[302, 651], [278, 674],[276, 497], [741, 673] ]
# t = [[601, 44],[601, 104], [128, 105], [58, 45], [126, 591], [59, 592], ]

# def turn(x):
#     list_x = []
#     list_y = []
#     X = []
#     Y = []
#     Z = []
#     for i in x:
#         list_x.append(i[0])
#         list_y.append(i[1])
print((1,2)+(3,))
    # for i in range(1,len(list_x)):
    #     for j in range(0,len(list_x)):
    #         if abs(list_x[i] - list_x[j]) <= 3:
    #             print(i,j)
    #             if abs(list_y[i] - list_y[j]) > 10:
    #                 X.append((list_x[i],list_y[i]))
    #                 X.append((list_x[i-1],list_y[i-1]))
    #     list_x.pop(i-1)
    #     list_x.pop(i)

    # print(X)
    # del list_x[i-1:i],list_y[i-1:i]

    # for i in range(1,len(list_y)):
    #     if abs(list_y[i-1] - list_y[i]) <= 3:
    #         if  abs(list_x[i-1] - list_x[i]) > 10:
    #             Y.append((list_x[i],list_y[i]))
    #             Y.append((list_x[i-1],list_y[i-1]))   
    # del list_x[i-1:i],list_y[i-1:i]
    
    # for i in range(1,len(list_x)):
    #     if abs(list_y[i-1] - list_y[i]) > 10  and abs(list_x[i-1] - list_x[i]) > 10 :
    #         # if  abs(list_x[i-1] - list_x[i]) > 6:
    #         Z.append((list_x[i],list_y[i]))
    #         Z.append((list_x[i-1],list_y[i-1]))
    # # del list_x[i-1:i],list_y[i-1:i]
    
    # return X,Y,Z

# print(turn(x))