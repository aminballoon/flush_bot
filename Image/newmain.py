import cv2
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def find_color(img,x,y):
	return img[y,x]

def find_gradient(img,x,y):
	return 255 - img[y,x]
	
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

def top_to_bot(list_is):
    ans = list_is[1:len(list_is)]
    ans.append(list_is[0])
    return ans
	
def find_num_point(x1,y1,x2,y2):
    if abs( x1 - x2 ) >= abs( y1 - y2 ):
        return int(abs( x1 - x2 ))
    else:
        return int(abs( y1 - y2 ))

def sampling(x1,y1,x2,y2):
    dx = x2-x1
    dy = y2-y1
    if abs( x1 - x2 ) >= abs( y1 - y2 ):
        n = int(abs( x1 - x2 ) /16)
    else:
        n = int(abs( y1 - y2 ) /16)
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
            if Child_check_path:
                list_contours_path.append(hie.index(i))

def connected_symbol_path(list_box,cont = 20,plot = True):
    global contours,list_all_point_path, image 
    for j in list_contours_box_symbol:
        list_x = []
        list_y = []
        for i in contours[j]:
            list_x.append(i.tolist()[0][0])
            list_y.append(i.tolist()[0][1])
        diff_x = int((max(list_x)-min(list_x))/2)
        diff_y = int((max(list_y)-min(list_y))/2)
        X = int(min(list_x) + diff_x)
        Y = int(min(list_y) + diff_y)
        const_x = diff_x + cont
        const_y = diff_y + cont
        
        # cv2.circle(image,(center_x,center_y), 3, (0,0,255), -1)
        offset_right =  X + const_x
        offset_left = X - const_x
        offset_up = Y + const_y
        offset_down = Y - const_y

        if find_color( gray,offset_right,Y) != 255:
            for i in sampling(*(X,Y),*(offset_right , Y)):
                list_all_point_path.append(i+(128,))
                if plot == True:
                    cv2.circle(image,i,2,(0,0,255),-1)
            # cv2.line(image,(center_x,center_y),(center_x + cont_x , center_y),(0,0,255),3)

        if find_color(	gray,offset_left,Y) != 255:
            for i in sampling(*(X,Y),*(offset_left , Y)):
                list_all_point_path.append(i+(128,))
                if plot == True:
                    cv2.circle(image,i,2,(0,0,255),-1)
                
            # cv2.line(image,(center_x,center_y),(center_x - cont_x , center_y),(0,0,255),3)

        if find_color(	gray,	X,	offset_up) != 255:
            for i in sampling(*(X,Y),*(X , offset_up)):
                list_all_point_path.append(i+(128,))
                if plot == True:
                    cv2.circle(image,i,2,(0,0,255),-1)
                
            # cv2.line(image,(center_x,center_y),(center_x , center_y + cont_y),(0,0,255),3)

        if find_color(	gray,	X,	offset_down) != 255:
            for i in sampling(*(X,Y),*(X , offset_down)):
                if plot == True:
                    cv2.circle(image,i,2,(0,0,255),-1)
                list_all_point_path.append(i+(128,))
            # cv2.line(image,(center_x,center_y),(center_x , center_y - cont_y),(0,0,255),3)

def point_path(list_path,plot = True):
    global list_all_point_path ,image 
    for k in list_path:
        epsilon = 0.003 * cv2.arcLength(contours[k], True)
        approx = cv2.approxPolyDP(contours[k], epsilon, True)
        list_approx = []

        for j in approx.tolist():
            list_approx.append(j[0])
        
        if len(list_approx) >= 7:
            a = 0
            b = -1
            list_line = []
            for _ in range(int(len(list_approx)/2)):
                point_xy_midle = find_middle(*tuple(list_approx[a]), *tuple(list_approx[b]))
                # cv2.circle(image, point_xy_midle  , 3, (0, 255, 0), -1)
                list_line.append(point_xy_midle)
                a += 1
                b -= 1
            for i in range(1,len(list_line)):
                for j in sampling(*list_line[i-1],*list_line[i]):
                    list_all_point_path.append(j+(find_gradient(gray,*j),))
                    if plot == True:
                        cv2.circle(image,j,2,(0,0,255),-1)

        else:
            list_approx = top_to_bot(list_approx)
            a = 0
            b = -1
            list_line = []
            for _ in range(int(len(list_approx)/2)):
                point_xy_midle = find_middle(*tuple(list_approx[a]), *tuple(list_approx[b]))
                # cv2.circle(image, point_xy_midle  , 3, (0, 255, 0), -1)
                list_line.append(point_xy_midle)
                a += 1
                b -= 1
            
            for i in range(1,len(list_line)):
                for j in sampling(*list_line[i-1],*list_line[i]):
                    list_all_point_path.append(j+(find_gradient(gray,*j),))
                    if plot == True:
                        cv2.circle(image,j,2,(0,0,255),-1)
				
def plot3D(point):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i in point:
        ax.scatter(i[1],i[0],i[2])
    plt.show()
    
def normalize_contour(img):
    cnt, _ = cv2.findContours(img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    bounding_rect = cv2.boundingRect(cnt[0])
    img_cropped_bounding_rect = img[bounding_rect[1]:bounding_rect[1] + bounding_rect[3],
                                bounding_rect[0]:bounding_rect[0] + bounding_rect[2]]

    new_height = int((1.0 * img.shape[0])/img.shape[1] * 300)
    img_resized = cv2.resize(img_cropped_bounding_rect, (300, new_height))
    return img_resized

if __name__ == "__main__":
    list_all_point_path = []
    list_contours_symbol = []
    list_contours_box_symbol = []
    list_contours_path = []

    image = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\Moduel_image\test_field.jpg')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    edge = cv2.Canny(blur,40,100) 
    
    kernel = np.ones((3,3))
    dilation = cv2.dilate(edge,kernel,iterations = 1)
    closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
    # cv2.imshow("ssss",closing)
    # cv2.waitKey(0)
    # contours, hierarchy = cv2.findContours(closing , cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 
    # print("Number of Contours found = " + str(len(contours))) 
    # add_point_to_list(hierarchy.tolist()[0]) 
    # connected_symbol_path(list_contours_box_symbol,cont = 20,plot=True)
    # point_path(list_contours_path,plot=True)
    # cv2.imshow("kkkk",image) 
    # plot3D(list_all_point_path)
    cv2.imshow("s123sss",closing)
    cv2.imshow("ssss",normalize_contour(closing))
    

    cv2.waitKey(0)

    
    # cv2.waitKey(0)
    # print(list_all_point_path)
    # print(list_contours_symbol)
    # print(list_contours_box_symbol)
    # print(list_contours_path)
    