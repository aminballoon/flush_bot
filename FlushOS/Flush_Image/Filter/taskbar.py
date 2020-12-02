import cv2
import numpy as np
import json

with open(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_main\Flush_Image\Capture_image\Image\parameter.json') as json_file:
    data = json.load(json_file)
    Parametersy = (data['Parameter'][0])

image = cv2.imread(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_main\Flush_Image\Capture_image\Image\Image_Anti_Obstacle2020_12_02_22_35_33_912603.png')
# image = image[45:555, 45:555]
# image = cv2.medianBlur(image,9)
# kernel = np.array([[-1,-1,-1],
#                     [-1, 9,-1],
#                     [-1,-1,-1]])
# sharpened = cv2.filter2D(image, -1, kernel) 
# image=cv2.fastNlMeansDenoising(image, None, 10, 7, 21)

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
Kernel_blur = Parametersy['Kernel_blur']
cv2.createTrackbar("Kernel_blur","image_blur",3,7,update)

cv2.namedWindow("image_edge")
Canny_Thres_1 = Parametersy['Canny_Thres_1']
cv2.createTrackbar("Canny_Thres_1","image_edge",Canny_Thres_1,255,update)
Canny_Thres_2 = Parametersy['Canny_Thres_2']
cv2.createTrackbar("Canny_Thres_2","image_edge",Canny_Thres_2,255,update)

cv2.namedWindow("image_morp")
Kernel_morp = Parametersy['Kernel_morp']
cv2.createTrackbar("Kernel_morp","image_morp",Kernel_morp,7,update)
Mode_morp = Parametersy['Mode_morp']
cv2.createTrackbar("Mode_morp","image_morp",Mode_morp,3,update)
iterations = Parametersy['iterations']
cv2.createTrackbar("iterations","image_morp",iterations,4,update)

cv2.namedWindow("image_contour")
contour_use = []
# maxArea = int(resolution_X*resolution_Y/50)
maxArea = Parametersy['maxArea']
minArea = Parametersy['minArea']
# minArea = 0
cv2.createTrackbar("maxArea","image_contour",maxArea,20000,update)
cv2.createTrackbar("minArea","image_contour",minArea,25000,update)

cv2.namedWindow("image_save")
save = 0
cv2.createTrackbar("save","image_save",0,1,update)

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
    # print("Number of Contours found = " + str(len(contours))) 
    for i in range(0, len(contours)):
        area = int(cv2.contourArea(contours[i]))
        if area >= minArea:
            if area <= maxArea*100:
                contour_use.append(i)
    black_image = np.zeros((resolution_X, resolution_Y, 1), np.uint8)
    # print(Kernel_blur,Kernel_morp)
    for i in contour_use:
        cv2.drawContours(black_image,contours,i,(255,255,255))
    black_image = cv2.dilate(black_image,Kernel_morp_use,iterations = iterations)
    cv2.imshow("image_contour",black_image) 
    
    contour1, hie = cv2.findContours(black_image , cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    if len(contour1) > 0 :
        black_image1 = np.zeros((resolution_X, resolution_Y, 1), np.uint8)
        cv2.drawContours(black_image1,contour1,-1,(255,255,255))
        blacked = np.zeros((resolution_X, resolution_Y, 1), np.uint8)
        for i in range(len(contour1)):
            # print(i)
            filled_contour = cv2.fillPoly(blacked, [contour1[i]], color=(255,255,255))
            _,filled_path_binary = cv2.threshold(filled_contour,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            filled_path_binary = cv2.dilate(filled_path_binary,Kernel_morp_use,iterations = 5 )
        cv2.imshow("image_asdasdasd",filled_path_binary) 
        thin_image = cv2.ximgproc.thinning(filled_path_binary,thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
        cv2.imshow("image_save",thin_image)
    # black_image1 = cv2.dilate(black_image1,Kernel_morp_use,iterations = iterations)
    # cv2.imshow("image_save",black_image1) 
    # print(hie)
        add_point_to_list(hie.tolist()[0])
    # print(list_contours_box_symbol)
    # print("Kuy")
    # print(len(contour1))

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

    save = cv2.getTrackbarPos("save","image_save")
    print(Kernel_blur,Kernel_morp)


if save == 1:
    data = {}
    data['Parameter'] = []
    data['Parameter'].append({
        # 'Kernel_blur': (int(((Kernel_blur+1)/2)-1)),
        'Kernel_blur': ((cv2.getTrackbarPos("Kernel_blur","image_blur")+1)*2)-1,
        'Canny_Thres_1': Canny_Thres_1,
        'Canny_Thres_2': Canny_Thres_2,
        # 'Kernel_morp': (int(((Kernel_morp)/2)-1)),
        'Kernel_morp': ((cv2.getTrackbarPos("Kernel_morp","image_morp")+1)*2)-1,
        'Mode_morp': Mode_morp,
        'iterations': iterations,
        'minArea' : minArea,
        # 'maxArea': int(maxArea/100)
        'maxArea': int(maxArea)
    })
    
    with open(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_main\Flush_Image\Capture_image\Image\parameter.json', 'w') as outfile:
        json.dump(data, outfile)

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



