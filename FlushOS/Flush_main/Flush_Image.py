import cv2
import numpy as np
import glob
from cv2 import aruco
import matplotlib.pyplot as plt
import json
from math import sqrt
from MTM import matchTemplates
import skimage.exposure
from Flush_Communication import *
from skimage.morphology import skeletonize
import skimage.graph
from math import sqrt,pi,atan2,degrees,cos,sin

def Sam_OOk_isas(List_Point,thres):
    re = []
    ans = []
    for i in range(0,len(List_Point)):
        for j in range(i+1,len(List_Point)):
            if (abs(List_Point[i][0] - List_Point[j][0]) < thres) and (abs(List_Point[i][1] - List_Point[j][1]) < thres):
                re.append(j)
    for i in range(0,len(List_Point)):
        if i not in re:
            ans.append(List_Point[i])
    return ans

def Sam_OOk(List_Point,thres=10):
    re = []
    ans = []
    for i in range(0,len(List_Point)):
        for j in range(i+1,len(List_Point)):
            one = (List_Point[i][0] - List_Point[j][0])
            two = (List_Point[i][1] - List_Point[j][1])
            if (sqrt((one*one)+(two*two)) < thres) :
                re.append(j)
    for i in range(0,len(List_Point)):
        if i not in re:
            ans.append(List_Point[i])
    return ans

def find_color(img,x,y):
	return img[y,x]

def find_gradient(img,x,y,thres = 140):
    if 255 - img[y,x] > thres:
        return 255
    else:
	    return 155

	
def find_white_in_black(thin_img):
    pixels = np.argwhere(thin_img == 255)
    lines = []
    for i in pixels.tolist():
        data = (i[1],i[0])
        if data not in lines:
            lines.append(data)
    return lines

def XytoYX(x,y):
    return (y,x)

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

def Born_To_be_Flushbot(list_cmd):
    ans = []
    for i in range(len(list_cmd)-1):
        if i == 0:
            ans.append(list_cmd[0] + (0,))
        elif i == 1:
            ans.append((0,0,list_cmd[1][2]))
            ans.append(list_cmd[i])
        # if i == len(list_cmd)-2:
        #     ans.append(list_cmd[i])
        else:
            ans.append(list_cmd[i])
        # print(i , ans)
    ans.append(list_cmd[-1]+(list_cmd[-2][2],))
    return ans
    

def find_point_symbol(image,list_template):
    font = cv2.FONT_HERSHEY_SIMPLEX
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
    point_symbol = []
    for template in list_template:
        # template = cv2.resize(template, (80, 80)) 
        # template = cv2.flip(template, 1)
        # template = template[10:90, 10:90]
        # cv2.imshow("thin9",template) 
        # cv2.waitKey(0)
        listTemplate = [('template', template)]
        for i,angle in enumerate([90,180]):
            rotated = np.rot90(template, k=i+1) # NB: np.rotate not good here, turns into float!
            listTemplate.append( (str(angle), rotated ) )
        Hits = matchTemplates(listTemplate, image_gray, N_object=1, score_threshold=0.4, method=cv2.TM_CCOEFF_NORMED, maxOverlap=1)
        point = Hits.values.tolist()[0][1]
        # print(Hits)    
        if float((Hits.values.tolist()[0][2])) >= 0.6:
            point_plot = (int(point[0]+(point[2]/2)),int(point[1]+(point[3]/2)))
            point_symbol.append(point_plot)

    point_symbol = Sam_OOk_isas(point_symbol,thres=5)

    for point in point_symbol:
        cv2.putText(image, (str(point[0]) + ","+ str(point[1])), point, font, 0.5, (255,0,0), 2)    
        cv2.circle(image,point,2,(0,255,0),-1)
    # print(point_symbol)
    return point_symbol

def Targectory_Gen(x1,y1,x2,y2):
    P1 = (x1,y1)
    P2 = (x2,y2)
    delta_x = P2[0]-P1[0]
    delta_y = P2[1]-P1[1]
    r = sqrt(pow(delta_x,2)+pow(delta_y,2))
    
    Theta = atan2(delta_y,delta_x)
    Theta = int(degrees(Theta))
    if Theta < 0:
        Theta += 180 
    Theta = Theta/2
    return int(r),int(Theta), int(r*cos(Theta)) , int(r*sin(Theta))

def sampling(x1,y1,x2,y2,prescaler = 2):
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

def plot3D(point):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i in point:
        ax.scatter(i[1],i[0],i[2])
    plt.show()

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
            if ans in range(40,70):
                print(list_approx[i],j,ans)
                cv2.circle(image, (list_approx[i][0],list_approx[i][1])  , 3, (255, 255, 0), -1)
                cv2.circle(image, (j[0],j[1])  , 3, (0, 255, 255), -1)
                cv2.circle(image, find_middle(j[0],j[1],list_approx[i][0],list_approx[i][1])  , 3, (255, 255, 255), -1)

def shortest_path(start,end,binary):
            costs=np.where(binary,1,1000)
            path, cost = skimage.graph.route_through_array(costs, start=start, end=end, fully_connected=True)
            return path

def thin_point_path(image,list_path,resolution_X,resolution_Y,contours,Kernel_morp_use,draw = True):
    Thining_Image = []
    list_approx = []
    list_use = []
    list_line = []
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    black_image = np.zeros((resolution_X, resolution_Y, 1), np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    for i in list_path:
        blacked = np.zeros((resolution_X,resolution_Y , 1), np.uint8)
        cv2.drawContours(blacked,contours,i,(255,255,255))
        blacked = cv2.dilate(blacked,kernel,iterations = 3)
        filled_contour = cv2.fillPoly(blacked, [contours[i]], color=(255,255,255))
        _,filled_path_binary = cv2.threshold(filled_contour,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        filled_path_binary = cv2.dilate(filled_path_binary,Kernel_morp_use,iterations = 5 )
        cv2.imshow("thin_img",filled_path_binary) 
        cv2.waitKey(0) 
        thin_image = cv2.ximgproc.thinning(filled_path_binary,thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
        point_thin = find_white_in_black(thin_image)
        Thining_Image.append(thin_image)
        
        for k in point_thin:
            x = k[0]
            y = k[1]
            cv2.circle(image,(x,y),1,(0,255,255),-1)
            kernely =  [thin_image[y-1,x-1],thin_image[y,x-1],thin_image[y+1,x-1],
                        thin_image[y-1,x],thin_image[y,x],thin_image[y+1,x],
                        thin_image[y-1,x+1],thin_image[y,x+1],thin_image[y+1,x+1]] 

            offset = 5
            if sum(kernely) <= 511:
                list_line.append(k)

        # print(list_line)
        con, _ = cv2.findContours(thin_image , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
        epsilon = 0.005 * cv2.arcLength(con[0],closed=True)
        approx = cv2.approxPolyDP(con[0], epsilon, closed=True)
        

        for j in approx.tolist():
            list_approx.append(tuple(j[0]))
        
        list_approx = Sam_OOk_isas(list_approx,10)
        # print(list_approx)

        font = cv2.FONT_HERSHEY_SIMPLEX
        for p in list_approx:
            cv2.circle(image,(p[0],p[1]),4,(241,12,255),-1)
            cv2.putText(image, (str(p[0]) + ","+ str(p[1])), (p[0],p[1]), font, 0.4, (0,0,255), 2)
        # for o in list_line:
        #     cv2.circle(image,(o[0],o[1]),4,(0,0,255),-1)
        #     cv2.putText(image, (str(o[0]) + ","+ str(o[1])), (o[0],o[1]), font, 0.3, (255,0,0), 2)
            # print((p[0],p[1],find_gradient(image_gray,p[0],p[1])))
        # list_approx += list_line
        # print(list_approx)
    return thin_image , list_approx , list_line , Thining_Image
        

        # path=(shortest_path((324,100),(145-20,285-+0),thin_image))
        # print(path)
        # blackeded = np.zeros((resolution_X,resolution_Y , 1), np.uint8)
        # for i in path:
        #     cv2.circle(blackeded,(i[1],i[0]),1,(255,255,255),-1)
        # cv2.imshow("sasdd",thin_image)
        # cv2.imshow("sd",blackeded)
        # cv2.waitKey(0)

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

# def shortest_path(start,end,binary):
#             costs=np.where(binary,1,1000)
#             path, cost = skimage.graph.route_through_array(costs, start=start, end=end, fully_connected=True)
#             return path
def Flush_Point_to_Control_1Thin(Flush_Point_Symbol,Flush_Point_Path,Flush_End_Point,Flush_Point_Control,list_line,resolution_X,resolution_Y,Thining_Image,Image_Gray):
    # for i in list_line:
    # print("Flush_Point_Control")
    # print(Flush_Point_Control)
    # print("list_line")
    # print(list_line)
    for i in list_line:
        if i not in Flush_Point_Control:
            Flush_Point_Control.append(i)
    # Flush_Point_Control = Sam_OOk(Flush_Point_Control,thres=3)
    # list_line = Sam_OOk(list_line,thres=1)
    Pathisas = []
    Cons = []
    Delta_x1 = (Flush_Point_Symbol[0][0] - list_line[0][0])
    Delta_y1 = (Flush_Point_Symbol[0][1] - list_line[0][1])
    Delta_x2 = (Flush_Point_Symbol[0][0] - list_line[1][0])
    Delta_y2 = (Flush_Point_Symbol[0][1] - list_line[1][1])
    if len(Flush_Point_Symbol) >= 2:
        Flush_End_Point = Flush_Point_Symbol[-1]
    else:
        if abs(sqrt(pow(Delta_x1+Delta_y1,2))) > abs(sqrt(pow(Delta_x2+Delta_y2,2))):
            Flush_End_Point = list_line[1]
        else:
            Flush_End_Point = list_line[0]
    path = shortest_path((Flush_Point_Symbol[0][1],Flush_Point_Symbol[0][0]),(Flush_End_Point[1],Flush_End_Point[0]),Flush_Point_Path)
    blackeded = np.zeros((resolution_X,resolution_Y , 1), np.uint8)
        # if Flush_Point_Symbol[0][0] - list_line[0]
    # for i in path:
    #     cv2.circle(blackeded,(i[1],i[0]),1,(255,255,255),-1)
    
    # cv2.imshow("sssssasdd",Flush_Point_Path)
    # cv2.imshow("sasdd",blackeded)
    # print(Flush_Point_Control)
    # print(path)
    # print(path)
    for i in Flush_Point_Control:
        if XytoYX(*tuple(i)) in path:
            Cons.append([path.index(XytoYX(*tuple(i))),i])
        # print(XytoYX(*tuple(i)))
    Pathisas.append(Flush_Point_Symbol[0])
    for i in sorted(Cons):
        Pathisas.append(i[1] + (find_gradient(Image_Gray,*i[1]),))

    Pathisas.append(Flush_End_Point)
    Pathisas = Sam_OOk(Pathisas,thres=1)

    return Pathisas


def Theta_isas(List_of_Position):
    ans = []
    _,Theta,_,_ = Targectory_Gen(List_of_Position[0][1],List_of_Position[0][0],List_of_Position[2][1],List_of_Position[2][0])
    ans.append(List_of_Position[0]+(Theta,))
    ans.append(List_of_Position[1]+(Theta,))
    for i in range(2,len(List_of_Position)-1):
        print(i,i+1)
        _,Theta,_,_ = Targectory_Gen(List_of_Position[i][1],List_of_Position[i][0],List_of_Position[i+1][1],List_of_Position[i+1][0])
        ans.append(List_of_Position[i]+(Theta,))
    _,Theta,_,_ = Targectory_Gen(List_of_Position[-2][1],List_of_Position[-2][0],List_of_Position[-1][1],List_of_Position[-1][0])
    ans.append(List_of_Position[-1]+(Theta,))
    return ans
    
def Flush_ImageProcessing(image,list_symbol_template,method = 'thining'):
    with open(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_main\Flush_Image\Flush_Parameter.json') as json_file:
        data = json.load(json_file)
    Parametersy = (data['Parameter'][0])
    Point_Symbol = []
    Point_Path = []
    
    # image = cv2.fastNlMeansDenoising(image, None, 10, 7, 21)

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
    blackedqq = DrawContours_on_Blacked_image(contours,contour_use,resolution_X,resolution_Y)
    cv2.imshow("ssdsw",blackedqq)
    black_image = np.zeros((resolution_X, resolution_Y, 1), np.uint8)

    Flush_Point_Symbol = find_point_symbol(image,list_symbol_template)
    # print(Flush_Point_Symbol)
    if method == 'thining':
        Flush_Point_Path,Flush_Point_Control,list_line ,Thining_Image= thin_point_path(image,contour_use,resolution_X,resolution_Y,contours,Kernel_morp_use,draw=True)
    else:
        point_path_conner(image,contours,contour_use,draw = True)
    # fill_path_image = cv2.fillPoly(black_image, [contours[1]], color=(255,255,255))
    Pathisas = Flush_Point_to_Control_1Thin(Flush_Point_Symbol,Flush_Point_Path,Flush_Point_Path[-1],Flush_Point_Control,list_line,resolution_X,resolution_Y,Thining_Image,image_gray)
    # Pathisas = Flush_Point_to_Control_2Thin(Flush_Point_Symbol,Flush_Point_Path,Flush_Point_Path[-1],Flush_Point_Control,list_line,resolution_X,resolution_Y,Thining_Image)
    # Pathisas = Sam_OOk(Pathisas,thres=5)
    
    Pathisas = Born_To_be_Flushbot(Pathisas)
    Pathisas = Theta_isas(Pathisas)
    print(Pathisas)

    # for q in [(54 , 330),(112 , 329),(239 , 325),(331 , 236),(331 , 112),(331 , 53),(270 , 54),(59 , 57),(57 , 191),(114 , 193),(180 , 191)]:
    #     print(q + (find_gradient(image_gray,*q),))
    return image 

    # _,filled_path_binary = cv2.threshold(fill_path_image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # kuy11 , kuyhie = cv2.findContours(filled_path_binary , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
    


