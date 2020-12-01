import cv2
import numpy as np
import glob
from cv2 import aruco
import matplotlib.pyplot as plt
import json
from math import sqrt
def find_color(img,x,y):
	return img[y,x]

def find_gradient(img,x,y):
	return 255 - img[y,x]
	
def find_white_in_black(thin_img):
    pixels = np.argwhere(thin_img == 255)
    lines = []
    for i in pixels.tolist():
        data = (i[1],i[0])
        if data not in lines:
            lines.append(data)
    return lines

def remove_duplicates(x):
  return list(dict.fromkeys(x))

def find_middle(x1, y1, x2, y2):
	if x1 >= x2:
		max_x = x1
		min_x = x2
	else:
		max_x = x2
		min_x = x1

	if y1 >= y2:
		max_y = y1
		min_y = y2
	else:
		max_y = y2
		min_y = y1
	xx = int(min_x + ((max_x-min_x)/2))
	yy = int(min_y + ((max_y-min_y)/2))
	return (xx,yy)

def find_point_symbol(list_template):
    for template in list_template:
        listTemplate = [('template', template)]
        for i,angle in enumerate([90,180]):
            rotated = np.rot90(template, k=i+1) # NB: np.rotate not good here, turns into float!
            listTemplate.append( (str(angle), rotated ) )
        Hits = matchTemplates(listTemplate, image_gray, N_object=1, score_threshold=0.6, method=cv2.TM_CCOEFF_NORMED, maxOverlap=0.3)
        point = Hits.values.tolist()[0][1]
        point_plot = (int(point[0]+(point[2]/2)),int(point[1]+(point[3]/2)))

        cv2.circle(image,point_plot,2,(0,255,0),-1)

def sampling(x1,y1,x2,y2,prescaler=2):
    dx = x2-x1
    dy = y2-y1
    if abs( x1 - x2 ) >= abs( y1 - y2 ):
        n = int(abs( x1 - x2 ) /prescaler)
    else:
        n = int(abs( y1 - y2 ) /prescaler)
    if n == 0:
        n = 1
    dxn = dx/n
    dyn = dy/n 
    output = []
    for i in range(n):
        output.append((int(x1+(i*dxn)),int(y1+(i*dyn))))
    return output

def add_point_to_list(hie):
    global list_contours_symbol, list_contours_box_symbol, list_contours_path
    for i in hie:  #ADD list_contours
        Parent_check = (i[3] == 0) # check in case
        Child_check_symbol = (i[2] != -1) # Symbol
        Child_check_path = (i[2] == -1) # PATH
        if Parent_check:
            if Child_check_symbol:
                list_contours_box_symbol.append(hie.index(i))
            # if Child_check_path:
            #     area = int(cv2.contourArea(contours[hie.index(i)]))
            #     print(str(hie.index(i)) +"   "+str(area))
            #     if area in range(0 , 50600):
            #         list_contours_path.append(hie.index(i))

def point_path_conner(image,contours,list_path,draw = True):
    for k in list_path:
        epsilon = 0.02 * cv2.arcLength(contours[k], True)
        approx = cv2.approxPolyDP(contours[k], epsilon, True)
        list_approx = []
        for j in approx.tolist():
            list_approx.append(j[0])
            # print(j[0])
            # cv2.circle(image, find_middle(j[0][0],j[0][1])  , 3, (0, 255, 0), -1)
        print(list_approx)
    for i in range(len(list_approx)):
        for j in list_approx:
            x = list_approx[i][0] - j[0]
            y = list_approx[i][1] - j[1]
            ans = int(sqrt((y*y)+(x*x)))
            if ans in range(30,50):
                print(list_approx[i],j,ans)
                cv2.circle(image, (list_approx[i][0],list_approx[i][1])  , 3, (255, 255, 0), -1)
                cv2.circle(image, (j[0],j[1])  , 3, (0, 255, 255), -1)
                cv2.circle(image, find_middle(j[0],j[1],list_approx[i][0],list_approx[i][1])  , 3, (255, 255, 255), -1)

def plot3D(point):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i in point:
        ax.scatter(i[1],i[0],i[2])
    plt.show()

def thin_point_path(image,list_path,resolution_X,resolution_Y,contours,Kernel_morp_use,draw = True):
    # global list_all_point_path 
    list_line = []
    black_image = np.zeros((resolution_X, resolution_Y, 1), np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    for i in list_path:
        blacked = np.zeros((resolution_X,resolution_Y , 1), np.uint8)
        cv2.drawContours(blacked,contours,i,(255,255,255))
        blacked = cv2.dilate(blacked,kernel,iterations = 3)
        filled_contour = cv2.fillPoly(blacked, [contours[i]], color=(255,255,255))
        _,filled_path_binary = cv2.threshold(filled_contour,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        filled_path_binary = cv2.dilate(filled_path_binary,Kernel_morp_use,iterations = 5 )
        thin_image = cv2.ximgproc.thinning(filled_path_binary,thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
        point_thin = find_white_in_black(thin_image)
        cv2.imwrite("img.png",thin_image) 
        for k in point_thin:
            x = k[0]
            y = k[1]
            cv2.circle(image,(x,y),1,(0,255,255),-1)
            kernely =  [thin_image[y-1,x-1],thin_image[y,x-1],thin_image[y+1,x-1],
                        thin_image[y-1,x],thin_image[y,x],thin_image[y+1,x],
                        thin_image[y-1,x+1],thin_image[y,x+1],thin_image[y+1,x+1]] 
            if sum(kernely) <= 510:
                cv2.circle(image,(x,y),4,(0,0,255),-1)
                

    # for i in list_line:
    #     for w in range(0,len(i),Set_Prescaler):
    #         # cv2.circle(image,i[w],2,(0,255,255),-1)
    #         list_all_point_path.append([*i[w],find_gradient(image_gray,*i[w])])

def DrawContours_on_Blacked_image(contours,List_number_of_contour,res_X,res_Y):
    black_image = np.zeros((res_X, res_Y, 1), np.uint8)
    for i in List_number_of_contour:
        cv2.drawContours(black_image,contours,i,(255,255,255))
    #cv2.dilate(black_image,Kernel_morp_use,iterations = 3) 
    return black_image

    
def Screening_PATH(contours,MinArea,MaxArea):
    list_contours_path = []
    for i in range(0, len(contours)):
        area = int(cv2.contourArea(contours[i]))
        if area >= MinArea:
            if area <= MaxArea*100:
                # contour_use.append(i)
                list_contours_path.append(i)
    return list_contours_path

def Flush_ImageProcessing(image,Path_symbol):
    with open(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_Image\Moduel_image\parameter.json') as json_file:
        data = json.load(json_file)
    Parametersy = (data['Parameter'][0])
    
    image = cv2.fastNlMeansDenoising(image, None, 10, 7, 21)

    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    Kernel_blur = Parametersy['Kernel_blur']
    image_blur = cv2.GaussianBlur(image_gray , (Kernel_blur,Kernel_blur), 0)

    Canny_Thres_1 = Parametersy['Canny_Thres_1']
    Canny_Thres_2 = Parametersy['Canny_Thres_2']
    image_edge = cv2.Canny(image_blur,Canny_Thres_1,Canny_Thres_2)

    Set_Prescaler = 16

    Kernel_morp = Parametersy['Kernel_morp']
    Kernel_morp_use = cv2.getStructuringElement(cv2.MORPH_CROSS, (Kernel_morp,Kernel_morp))
    iterations = Parametersy['iterations']
    image_dilation = cv2.dilate(image_edge,Kernel_morp_use ,iterations = iterations)
    image_closing = cv2.morphologyEx(image_dilation, cv2.MORPH_CROSS, Kernel_morp_use ) #cv2.MORPH_CROSS
    contours, hierarchy = cv2.findContours(image_closing , cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 
    resolution_X = image.shape[0]
    resolution_Y = image.shape[1]
    
    maxArea = Parametersy['maxArea']
    minArea = Parametersy['minArea']
    contour_use = Screening_PATH(contours,minArea,maxArea)
    DrawContours_on_Blacked_image(contours,contour_use,resolution_X,resolution_Y)
    black_image = np.zeros((resolution_X, resolution_Y, 1), np.uint8)
    # thin_point_path(image,contour_use,resolution_X,resolution_Y,contours,Kernel_morp_use,draw=True)
    point_path_conner(image,contours,contour_use,draw = True)
    # fill_path_image = cv2.fillPoly(black_image, [contours[1]], color=(255,255,255))
    
    return image 

    # _,filled_path_binary = cv2.threshold(fill_path_image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # kuy11 , kuyhie = cv2.findContours(filled_path_binary , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
    



    
