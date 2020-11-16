import cv2
import numpy as np
import json

image = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\Moduel_image\messageImage_1604660857654.jpg')
# image = cv2.medianBlur(image,9)
# kernel = np.array([[-1,-1,-1],
#                     [-1, 9,-1],
#                     [-1,-1,-1]])
# sharpened = cv2.filter2D(image, -1, kernel) 
# image=cv2.fastNlMeansDenoising(sharpened, None, 10, 2, 21)

image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
resolution_X = image.shape[0]
resolution_Y = image.shape[1]

def update(x):
    pass

def add_point_to_list(hie):
    global list_contours_symbol, list_contours_box_symbol, list_contours_path
    for i in hie:  #ADD list_contours
        Parent_check = (i[3] == 0) # check in case
        Child_check_symbol = (i[2] != -1) # Symbol
        Child_check_path = (i[2] == -1) # PATH
        if Parent_check:
            if Child_check_symbol:
                list_contours_box_symbol.append(hie.index(i))
            if Child_check_path:
                list_contours_path.append(hie.index(i))


cv2.namedWindow("image_blur")
Kernel_blur = 1
cv2.createTrackbar("Kernel_blur","image_blur",3,7,update)

cv2.namedWindow("image_edge")
Canny_Thres_1 = 40
cv2.createTrackbar("Canny_Thres_1","image_edge",Canny_Thres_1,255,update)
Canny_Thres_2 = 100
cv2.createTrackbar("Canny_Thres_2","image_edge",Canny_Thres_2,255,update)

cv2.namedWindow("image_morp")
Kernel_morp = 1
cv2.createTrackbar("Kernel_morp","image_morp",Kernel_morp,7,update)
Mode_morp = 1
cv2.createTrackbar("Mode_morp","image_morp",Mode_morp,3,update)
iterations = 1
cv2.createTrackbar("iterations","image_morp",iterations,4,update)

cv2.namedWindow("image_contour")
contour_use = []
maxArea = int(resolution_X*resolution_Y/50)
minArea = 0
cv2.createTrackbar("maxArea","image_contour",6045,20000,update)
cv2.createTrackbar("minArea","image_contour",3325,20000,update)

list_all_point_path = []
list_contours_symbol = []
list_contours_box_symbol = []
list_contours_path = []
list_point_connected = []

while(1):
    image_blur = cv2.GaussianBlur(image_gray , (Kernel_blur,Kernel_blur), 0)
    cv2.imshow("image_blur",image_blur)

    image_edge = cv2.Canny(image_blur,Canny_Thres_1,Canny_Thres_2)
    cv2.imshow("image_edge",image_edge) 
    Kernel_morp_use = cv2.getStructuringElement(cv2.MORPH_CROSS, (Kernel_morp,Kernel_morp))
    # Kernel_morp_use  = cv2.GaussianBlur(image_gray , (Kernel_morp,Kernel_morp), 0)
    image_morp = cv2.dilate(image_edge,Kernel_morp_use,iterations = iterations)
    if Mode_morp == 1:
        image_morp = cv2.morphologyEx(image_morp, cv2.MORPH_CLOSE, Kernel_morp_use)
    elif Mode_morp == 2:
        image_morp = cv2.morphologyEx(image_morp, cv2.MORPH_CROSS, Kernel_morp_use)
    elif Mode_morp == 3:
        image_morp = cv2.morphologyEx(image_morp, cv2.MORPH_OPEN, Kernel_morp_use)
    cv2.imshow("image_morp",image_morp) 
    
    contours, hierarchy = cv2.findContours(image_morp , cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for i in range(0, len(contours)):
        area = int(cv2.contourArea(contours[i]))
        if area >= minArea:
            if area <= maxArea*100:
                contour_use.append(i)
    black_image = np.zeros((resolution_X, resolution_Y, 1), np.uint8)

    for i in contour_use:
        cv2.drawContours(black_image,contours,i,(255,255,255))
    black_image = cv2.dilate(black_image,Kernel_morp_use,iterations = iterations)
    cv2.imshow("image_contour",black_image) 
    
    contour1, hie = cv2.findContours(black_image , cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    black_image1 = np.zeros((resolution_X, resolution_Y, 1), np.uint8)
    cv2.drawContours(black_image1,contour1,-1,(255,255,255))
    blacked = np.zeros((resolution_X, resolution_Y, 1), np.uint8)
    filled_contour = cv2.fillPoly(blacked, [contour1[1]], color=(255,255,255))
    _,filled_path_binary = cv2.threshold(filled_contour,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    thin_image = cv2.ximgproc.thinning(filled_path_binary,thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
    cv2.imshow("thin_image",thin_image)
    # black_image1 = cv2.dilate(black_image1,Kernel_morp_use,iterations = iterations)
    cv2.imshow("image_Kuy",black_image1) 
    # print(hie)
    add_point_to_list(hie.tolist()[0])
    # print(list_contours_box_symbol)
    # print("Kuy")
    print(len(contour1))

    contour_use = []
    list_contours_path = []
    list_contours_box_symbol = []
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
    Kernel_blur = ((cv2.getTrackbarPos("Kernel_blur","image_blur")+1)*2)-1
    
    Canny_Thres_1 = cv2.getTrackbarPos("Canny_Thres_1","image_edge")
    Canny_Thres_2 = cv2.getTrackbarPos("Canny_Thres_2","image_edge")

    Kernel_morp = ((cv2.getTrackbarPos("Kernel_morp","image_morp")+1)*2)-1
    Mode_morp = cv2.getTrackbarPos("Mode_morp","image_morp")
    iterations = cv2.getTrackbarPos("iterations","image_morp")
    
    minArea = cv2.getTrackbarPos("minArea","image_contour")
    maxArea = cv2.getTrackbarPos("maxArea","image_contour")

    data = {}
    data['Parameter'] = []
    data['Parameter'].append({
        'Kernel_blur': (int(((Kernel_blur+1)/2)-1)),
        'Canny_Thres_1': Canny_Thres_1,
        'Canny_Thres_2': Canny_Thres_2,
        'Kernel_morp': (int(((Kernel_morp+1)/2)-1)),
        'Mode_morp': Mode_morp,
        'iterations': iterations,
        'minArea:' : minArea,
        'maxArea': int(maxArea/100)
    })


    # data['blur'].append({
    #     'name': 'Larry',
    #     'website': 'google.com',
    #     'from': 'Michigan'
    # })
    # data['blur'].append({
    #     'name': 'Tim',
    #     'website': 'apple.com',
    #     'from': 'Alabama'
    # })

with open(r'C:\Users\aminb\Desktop\FIBO\Image\Moduel_image\parameter.json', 'w') as outfile:
    json.dump(data, outfile)

