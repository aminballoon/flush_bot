import cv2
import matplotlib.pyplot as plt
import numpy as np
from MTM import matchTemplates
from mpl_toolkits.mplot3d import Axes3D
import glob
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

def find_num_point(x1,y1,x2,y2):
    if abs( x1 - x2 ) >= abs( y1 - y2 ):
        return int(abs( x1 - x2 ))
    else:
        return int(abs( y1 - y2 ))

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

def connected_symbol_path(list_box,const = 30,draw = True):
    global contours,list_all_point_path, image ,filled_path_binary ,resolution_X,resolution_Y
    for j in list_contours_box_symbol:
        list_x = []
        list_y = []
        for i in contours[j]:
            list_x.append(i.tolist()[0][0])
            list_y.append(i.tolist()[0][1])
        max_x = int((max(list_x)))+1
        min_x = int((min(list_x)))-1
        max_y = int((max(list_y)))+1
        min_y = int((min(list_y)))-1
        diff_x = int((max(list_x)-min(list_x))/2)
        diff_y = int((max(list_y)-min(list_y))/2)
        X = int(min(list_x) + diff_x)
        Y = int(min(list_y) + diff_y)
        const_x = diff_x + const
        const_y = diff_y + const
        
        # cv2.circle(image,(center_x,center_y), 3, (0,0,255), -1)
        offset_right =  X + const_x
        offset_left = X - const_x
        offset_up = Y + const_y
        offset_down = Y - const_y
        
        cost = min([resolution_X,resolution_Y])

        if offset_right in range(1,cost):
            if find_color( filled_path_binary,offset_right,Y) >= 1:
                for i in sampling(*(X,Y),*(offset_right + (int(const_x/2)) , Y),prescaler=Set_Prescaler):
                    if i[0] in range(min_x,max_x):
                        list_all_point_path.append(i+(128,))
                    else:
                        list_all_point_path.append(i+(find_gradient(image_gray,*i),))
                    if draw == True:
                        cv2.circle(image,i,2,(0,0,255),-1)
                # cv2.line(image,(center_x,center_y),(center_x + cont_x , center_y),(0,0,255),3)

        if offset_left in range(1,cost):
            if find_color(	filled_path_binary,offset_left,Y) >= 1:
                for i in sampling(*(X,Y),*(offset_left - (int(const_x/2)) , Y),prescaler=Set_Prescaler):
                    if i[0] in range(min_x,max_x):
                        list_all_point_path.append(i+(128,))
                    else:
                        list_all_point_path.append(i+(find_gradient(image_gray,*i),))
                    if draw == True:
                        cv2.circle(image,i,2,(0,0,255),-1)
                    
                # cv2.line(image,(center_x,center_y),(center_x - cont_x , center_y),(0,0,255),3)
        if offset_up in range(1,cost):
            if find_color(	filled_path_binary,	X,	offset_up ) >= 1:
                for i in sampling(*(X,Y),*(X , offset_up + (int(const_y/2))),prescaler=Set_Prescaler):
                    if i[1] in range(min_y,max_y):
                        list_all_point_path.append(i+(128,))
                    else:
                        list_all_point_path.append(i+(find_gradient(image_gray,*i),))
                    if draw == True:
                        cv2.circle(image,i,2,(0,0,255),-1)
                
                # cv2.line(image,(center_x,center_y),(center_x , center_y + cont_y),(0,0,255),3)
        if offset_down in range(1,cost):
            if find_color(	filled_path_binary,	X,	offset_down ) >= 1:
                for i in sampling(*(X,Y),*(X , offset_down - (int(const_y/2))),prescaler=Set_Prescaler):
                    if i[1] in range(min_y,max_y):
                        list_all_point_path.append(i+(128,))
                    else:
                        list_all_point_path.append(i+(find_gradient(image_gray,*i),))
                    if draw == True:
                        cv2.circle(image,i,2,(0,0,255),-1)
                # cv2.line(image,(center_x,center_y),(center_x , center_y - cont_y),(0,0,255),3)

def point_path(list_path,draw = True):
    global list_all_point_path ,image 
    for k in list_path:
        epsilon = 0.02 * cv2.arcLength(contours[k], True)
        approx = cv2.approxPolyDP(contours[k], epsilon, True)
        list_approx = []
        o = 0
        for j in approx.tolist():
            list_approx.append(j[0])
            # print(j[0])
            cv2.circle(image, (j[0][0],j[0][1])  , 3, (0, 255, o), -1)
            o += 50
        print(list_approx)
        # if len(list_approx) >= 7:
        #     a = 0
        #     b = -1
        #     list_line = []
        #     for _ in range(int(len(list_approx)/2)):
        #         point_xy_midle = find_middle(*tuple(list_approx[a]), *tuple(list_approx[b]))
        #         # cv2.circle(image, point_xy_midle  , 3, (0, 255, 0), -1)
        #         list_line.append(point_xy_midle)
        #         a += 1
        #         b -= 1
        #     for i in range(1,len(list_line)):
        #         for j in sampling(*list_line[i-1],*list_line[i],prescaler=Set_Prescaler):
        #             list_all_point_path.append(j+(find_gradient(image_gray,*j),))
        #             if draw == True:
        #                 cv2.circle(image,j,2,(0,0,255),-1)

        # else:
        #     list_approx = top_to_bot(list_approx)
        #     a = 0
        #     b = -1
        #     list_line = []
        #     for _ in range(int(len(list_approx)/2)):
        #         point_xy_midle = find_middle(*tuple(list_approx[a]), *tuple(list_approx[b]))
        #         # cv2.circle(image, point_xy_midle  , 3, (0, 255, 0), -1)
        #         list_line.append(point_xy_midle)
        #         a += 1
        #         b -= 1
            
        #     for i in range(1,len(list_line)):
        #         for j in sampling(*list_line[i-1],*list_line[i],prescaler=Set_Prescaler):
        #             list_all_point_path.append(j+(find_gradient(image_gray,*j),))
        #             if draw == True:
        #                 cv2.circle(image,j,2,(0,0,255),-1)
				
def find_white_in_black(thin_img):
    pixels = np.argwhere(thin_img == 255)
    lines = []
    for i in pixels.tolist():
        data = (i[1],i[0])
        if data not in lines:
            lines.append(data)
    return lines

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

def thin_point_path(list_path,draw = True):
    global contours , list_all_point_path , image ,resolution_X,resolution_Y
    list_line = []
    black_image = np.zeros((resolution_X, resolution_Y, 1), np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    for i in list_path:
        blacked = np.zeros((resolution_X,resolution_Y , 1), np.uint8)
        cv2.drawContours(blacked,contours,i,(255,255,255))
        blacked = cv2.dilate(blacked,kernel,iterations = 3)
        # cv2.imshow("img_show",blacked)
        # cv2.waitKey(0) 
        filled_contour = cv2.fillPoly(blacked, [contours[i]], color=(255,255,255))
        _,filled_path_binary = cv2.threshold(filled_contour,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        filled_path_binary = cv2.dilate(filled_path_binary,Kernel_morp_use,iterations = 5 )
        thin_image = cv2.ximgproc.thinning(filled_path_binary,thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
        point_thin = find_white_in_black(thin_image)
        cv2.imwrite("img.png",thin_image) 
        for k in point_thin:
            x = k[0]
            y = k[1]
            cv2.circle(image,(x,y),2,(0,255,255),-1)
            kernely =  [thin_image[y-1,x-1],thin_image[y,x-1],thin_image[y+1,x-1],
                        thin_image[y-1,x],thin_image[y,x],thin_image[y+1,x],
                        thin_image[y-1,x+1],thin_image[y,x+1],thin_image[y+1,x+1]] 
            if sum(kernely) <= 510:
                cv2.circle(image,(x,y),2,(0,0,255),-1)
                

    for i in list_line:
        for w in range(0,len(i),Set_Prescaler):
            cv2.circle(image,i[w],2,(0,0,255),-1)
            list_all_point_path.append([*i[w],find_gradient(image_gray,*i[w])])

def new_find_point_symbol():
    global image_gray,list_symbol_template
    sift = cv2.xfeatures2d.SIFT_create(nfeatures=100)
    for symbol in list_symbol_template:
        keypoints_1, descriptors_1 = sift.detectAndCompute(image_gray,None)
        keypoints_2, descriptors_2 = sift.detectAndCompute(symbol,None)
        bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)
        matches = bf.match(descriptors_1,descriptors_2)
        print("symbol" + str(list_symbol_template.index(symbol)))
        print(len(matches))
        for i in matches:
            y,x = keypoints_1[i.trainIdx].pt
            x = int(x)
            y = int(y)
            cv2.circle(image,(x,y),2,(0,0,255),-1)
            cv2.imshow("img_show",image) 
            cv2.waitKey(0)
        #     for j in list_contours_box_symbol:
        #         if (x,y) in contours[j]:
        #             print(x,y)
        #             cv2.circle(image,(x,y),2,(0,0,255),-1)
        #             cv2.imshow("img_show",image) 
        #             cv2.waitKey(0)
        img3 = cv2.drawMatches(image_gray, keypoints_1, symbol, keypoints_2, matches, symbol, flags=2)
        cv2.imshow("swqe",img3)


# def new_connected_symbol_path()
#     global list_all_point_path ,image ,list_contours_box_symbol ,list_contours_path
    

def auto_canny(image, sigma=0.33):
	# compute the median of the single channel pixel intensities
	v = np.median(image)
	# apply automatic Canny edge detection using the computed median
	lower = int(max(0, (1.0 - sigma) * v))
	upper = int(min(255, (1.0 + sigma) * v))
	edged = cv2.Canny(image, lower, upper)
	# return the edged image
	return edged

if __name__ == "__main__":
    # list_symbol_template = []
    list_all_point_path = []
    list_contours_symbol = []
    list_contours_box_symbol = []
    list_contours_path = []
    list_point_connected = []
    list_symbol_template = [cv2.imread(file,0) for file in glob.glob(r'C:\Users\aminb\Desktop\FIBO\Image\symbol_module\*.jpg')]

    image = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\Moduel_image\ready_field(1).jpg')

    # kernel = np.array([[-1,-1,-1],
    #                 [-1, 9,-1],
    #                 [-1,-1,-1]])
    # sharpened = cv2.filter2D(image, -1, kernel)
    image=cv2.fastNlMeansDenoising(image, None, 10, 7, 21)
    
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # gray_filtered = cv2.bilateralFilter(image_gray, 7, 50, 50)
    # image_blur = cv2.GaussianBlur(image_gray, (5,5), 0)
    image_blur = cv2.GaussianBlur(image_gray , (7,7), 0)
    # eie = cv2.subtract(image_blur,image_gray)
    # _,thresh1 = cv2.threshold(eie,3,255,cv2.THRESH_BINARY)
    
    image_edge = cv2.Canny(image_blur,40,100) 
    # cv2.imshow("image_cssdasdontour",image_edge) 
    # cv2.waitKey(0)
    # image_edge = cv2.Canny(image_blur,10,100)

    Set_Prescaler = 16

    # kernel = np.ones((3,3))
    Kernel_morp_use = cv2.getStructuringElement(cv2.MORPH_CROSS, (7,7))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    image_dilation = cv2.dilate(image_edge,Kernel_morp_use ,iterations = 2)
    image_closing = cv2.morphologyEx(image_dilation, cv2.MORPH_CROSS, Kernel_morp_use ) #cv2.MORPH_CROSS
    
    # cv2.imshow("img_shoqwewwww",image_closing) 
    # cv2.waitKey(0)

    contours, hierarchy = cv2.findContours(image_closing , cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 

    resolution_X = image.shape[0]
    resolution_Y = image.shape[1]
    print(resolution_X,resolution_Y)
    contour_use = []
    for i in range(0, len(contours)):
        area = int(cv2.contourArea(contours[i]))
        if area >= 16196:
            if area <= 406*100:
                contour_use.append(i)
                list_contours_path.append(i)

    black_image4 = np.zeros((resolution_X, resolution_Y, 1), np.uint8)
    
    for i in contour_use:
        cv2.drawContours(black_image4,contours,i,(255,255,255))
    black_image4 = cv2.dilate(black_image4,Kernel_morp_use,iterations = 3)



    black_image = np.zeros((resolution_X, resolution_Y, 1), np.uint8)
    fill_path_image = cv2.fillPoly(black_image, [contours[1]], color=(255,255,255))
    _,filled_path_binary = cv2.threshold(fill_path_image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    kuy11 , kuyhie = cv2.findContours(filled_path_binary , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
    print("Number of Contours found = " + str(len(contours))) 
    
    # for i in range(0,len(contours)):
    #     sssse = np.zeros((resolution_X, resolution_Y, 1), np.uint8)
    #     cv2.drawContours(sssse,contours,i,(0,0,255))
    black_imasssge = np.zeros((resolution_X, resolution_Y, 1), np.uint8)
    black_imasssge = cv2.drawContours(black_imasssge,kuy11,-1,(255,255,255))


    add_point_to_list(hierarchy.tolist()[0]) 
    connected_symbol_path(list_contours_box_symbol,const = 30,draw=True)
    # find_point_symbol(list_symbol_template)
    # new_find_point_symbol()
    # print(list_contours_path)
    # thin_point_path(list_contours_path,draw=True)
    point_path(list_contours_path,draw=True)

    cv2.imshow("img_show",image) 
    # plot3D(list_all_point_path)
    cv2.waitKey(0)
    cv2.destroyAllWindows() 


    # cv2.imshow("img_show",image) 
    # cv2.waitKey(0)
    # print(list_all_point_path)
    # print(list_contours_symbol)
    # print(list_contours_box_symbol)
    # print(list_contours_path)
    