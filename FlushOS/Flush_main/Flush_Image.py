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

def Sam_OOk_isas(List_Point,thres=5):
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

def find_gradient(img,x,y,thres = 140): # 140
    if 255 - img[y,x] > thres:
        return 250
    else:
	    return 160

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
        elif len(list_cmd[i]) == 2 and list_cmd[i] !=  list_cmd[-1]:
            ans.append(list_cmd[i]+(list_cmd[i-1][2],))
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
    # Theta = (Theta * 180 / 3.1412)
    Theta = degrees(Theta)
    if Theta < 0:
        Theta += 180 
    Theta = (Theta/2)
    return int(r),90 - int(Theta), int(r*cos(Theta)) , int(r*sin(Theta))

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
    Flush_Point_Path = []
    
    list_use = []
    list_conner = []
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    black_image = np.zeros((resolution_X, resolution_Y, 1), np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    for i in list_path:
        list_approx = []
        list_conner_path = []
        blacked = np.zeros((resolution_X,resolution_Y , 1), np.uint8)
        cv2.drawContours(blacked,contours,i,(255,255,255))
        blacked = cv2.dilate(blacked,kernel,iterations = 3)
        filled_contour = cv2.fillPoly(blacked, [contours[i]], color=(255,255,255))
        _,filled_path_binary = cv2.threshold(filled_contour,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        filled_path_binary = cv2.dilate(filled_path_binary,Kernel_morp_use,iterations = 5 )
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
                list_conner_path.append(k)

        # print(list_conner_path)
        con, _ = cv2.findContours(thin_image , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
        epsilon = 0.005 * cv2.arcLength(con[0],closed=True)
        approx = cv2.approxPolyDP(con[0], epsilon, closed=True)
        
        for j in approx.tolist():
            list_approx.append(tuple(j[0]))
        
        list_approx = Sam_OOk_isas(list_approx,20)
        # print(list_approx)

        font = cv2.FONT_HERSHEY_SIMPLEX
        for p in list_approx:
            cv2.circle(image,(p[0],p[1]),4,(241,12,255),-1)
            cv2.putText(image, (str(p[0]) + ","+ str(p[1])), (p[0],p[1]), font, 0.4, (0,0,255), 2)
        # cv2.imshow("oooo",image)
        # cv2.waitKey(0)
        list_conner.append(list_conner_path)
        Flush_Point_Path.append(list_approx)
    Flush_Point_Conner = list_conner 
    # list_approx = Sam_OOk_isas(list_approx)
    return Thining_Image , Flush_Point_Path , Flush_Point_Conner
        


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


def Flush_Point_to_Control_1Thin(Flush_Point_Symbol,Flush_Point_Path,Flush_End_Point,Flush_Point_Control,list_conner,resolution_X,resolution_Y,Thining_Image,Image_Gray):

    for i in list_conner:
        if i not in Flush_Point_Control:
            Flush_Point_Control.append(i)
    # Flush_Point_Control = Sam_OOk(Flush_Point_Control,thres=3)
    # list_conner = Sam_OOk(list_conner,thres=1)
    Pathisas = []
    Cons = []
    Delta_x1 = (Flush_Point_Symbol[0][0] - list_conner[0][0])
    Delta_y1 = (Flush_Point_Symbol[0][1] - list_conner[0][1])
    Delta_x2 = (Flush_Point_Symbol[0][0] - list_conner[1][0])
    Delta_y2 = (Flush_Point_Symbol[0][1] - list_conner[1][1])

    if len(Flush_Point_Symbol) >= 2:
        Flush_End_Point = Flush_Point_Symbol[-1]
    else:
        if abs(sqrt(pow(Delta_x1+Delta_y1,2))) > abs(sqrt(pow(Delta_x2+Delta_y2,2))):
            Flush_End_Point = list_conner[1]
        else:
            Flush_End_Point = list_conner[0]
    path = shortest_path((Flush_Point_Symbol[0][1],Flush_Point_Symbol[0][0]),(Flush_End_Point[1],Flush_End_Point[0]),Flush_Point_Path)
    blackeded = np.zeros((resolution_X,resolution_Y , 1), np.uint8)
        # if Flush_Point_Symbol[0][0] - list_conner[0]
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
    # Pathisas = Sam_OOk(Pathisas,thres=1)

    return Pathisas

def Theta_isas(List_of_Position):
    ans = []
    _,Theta,_,_ = Targectory_Gen(List_of_Position[0][1],List_of_Position[0][0],List_of_Position[2][1],List_of_Position[2][0])
    ans.append(List_of_Position[0]+(Theta,))
    ans.append(List_of_Position[1]+(Theta,))
    for i in range(2,len(List_of_Position)-1):
        # print(i,i+1)
        _,Theta,_,_ = Targectory_Gen(List_of_Position[i][1],List_of_Position[i][0],List_of_Position[i+1][1],List_of_Position[i+1][0])
        ans.append(List_of_Position[i]+(Theta,))
    _,Theta,_,_ = Targectory_Gen(List_of_Position[-2][1],List_of_Position[-2][0],List_of_Position[-1][1],List_of_Position[-1][0])
    ans.append(List_of_Position[-1]+(Theta,))
    return ans
    

def Flush_Sorted_Symbol_Point(List_Symbol,Find):
    Symbol1 = []
    Symbol2 = []
    for i in List_Symbol:
        if Find in i:
            num = i.index(Find)
            Symbol1 = [i[num],i[num-1]]
        else:
            Symbol2 = i
    return Symbol1,Symbol2

def XYtoYX_List(Listy):
    path = []
    for i in Listy:
        path.append((i[1],i[0]))
    return path

def Flush_Point_to_Control_isas(Thining_Image,Flush_Point_Symbol,Flush_Point_Path,Flush_Point_Conner,resolution_X,resolution_Y,Image_Gray):
    # print(Flush_Point_Conner)
    Cons = []
    Cons1 = []
    Pathisas = []
    # print(Flush_Point_Conner)
    Pathisas.append(Flush_Point_Symbol[0])
    x0 = Flush_Point_Symbol[0][0]
    y0 = Flush_Point_Symbol[0][1]
    Start_point_0 = []
    Start_point_1 = []
    for i in Flush_Point_Conner:
        for x1,y1 in i:
            Delta_x = int((x0-x1)**2)
            Delta_y = int((y0-y1)**2)
            Start_point_0.append([int(sqrt(Delta_x+Delta_y)),(x1,y1)])
    Start0,Start1 = Flush_Sorted_Symbol_Point(Flush_Point_Conner,min(Start_point_0)[1])
    
    First_Path = find_white_in_black(Thining_Image[0])
    if len(Thining_Image) > 1:
        if Start0[0] in First_Path:
            Thining_Image_Used = Thining_Image[0]
            indexy = 0
        else:
            Thining_Image_Used = Thining_Image[1]
            indexy = 1
    else:
        Thining_Image_Used = Thining_Image[0]
        indexy = 0


    symbol0_x = Flush_Point_Symbol[0][0]
    symbol0_y = Flush_Point_Symbol[0][1]
    symbol1_x = Flush_Point_Symbol[1][0]
    symbol1_y = Flush_Point_Symbol[1][1]
    if len(Flush_Point_Symbol) == 3:
        symbol2_x = Flush_Point_Symbol[2][0]
        symbol2_y = Flush_Point_Symbol[2][1]
    path = shortest_path((symbol0_y,symbol0_x),(symbol1_y,symbol1_x),Thining_Image_Used)
    
    # black_image = np.zeros((resolution_X, resolution_Y, 1), np.uint8)
    # for i in path:
    #     cv2.circle(black_image,i,1,(255,255,255))
    #     cv2.imshow("ppp",black_image)
    #     cv2.waitKey(0)
    # print(path[0])
    for i in Flush_Point_Path[indexy]:
        
        if XytoYX(*tuple(i)) in path:
            
            Cons.append([path.index(XytoYX(*tuple(i))),i])
        else:
            xy = XytoYX(*tuple(i)) 
            dist = lambda x, y: (x[0]-y[0])**2 + (x[1]-y[1])**2
            o = min(path, key=lambda co: dist(co, xy))
            Cons1.append([path.index(o),i])
               

    # for i in Flush_Point_Path[indexy]:
    #     if i in path:
    #         Cons.append([path.index(i),i])
    # print(Cons)
    for i in sorted(Cons):
        Pathisas.append(i[1] + (find_gradient(Image_Gray,*i[1]),))
    # print(Pathisas)
    Pathisas.append(Flush_Point_Symbol[1])
    if len(Thining_Image) > 1:
        x1 = Flush_Point_Symbol[1][0]
        y1 = Flush_Point_Symbol[1][1]
        # print(Start1)
        # print(Flush_Point_Conner)
        for x2,y2 in Start1:
            Delta_x = int((x1-x2)**2)
            Delta_y = int((y1-y2)**2)
            # print(i)
            # for x2,y2 in i:
            Start_point_1.append([int(sqrt(Delta_x+Delta_y)),(x2,y2)])
        Start1 = min(Start_point_1)[1]
        if indexy == 0:
            indexy = 1
            Thining_Image_Used = Thining_Image[indexy]
        else:
            indexy = 0
            Thining_Image_Used = Thining_Image[indexy]
        
        # area = [[1,1],[1,0],[0,1],[-1,-1],[-1,1],[1,-1],[-1,0],[0,-1],[,],[,],[,],[,],[,],[,],[,],[,],[,],[,],[,],[,],[,],[,],[,],[,],[,]]
        path = shortest_path((symbol1_y,symbol1_x),(symbol2_y,symbol2_x),Thining_Image_Used)
        for i in Flush_Point_Path[indexy]:
            # print(XytoYX(*tuple(i)))
            if XytoYX(*tuple(i)) in path:
                Cons1.append([path.index(XytoYX(*tuple(i))),i])
            else:
                xy = XytoYX(*tuple(i)) 
                dist = lambda x, y: (x[0]-y[0])**2 + (x[1]-y[1])**2
                o = min(path, key=lambda co: dist(co, xy))
                Cons1.append([path.index(o),i])
                # for k in [[-1,-1],[1,-1],[-1,1],[1,1],[-1,0],[1,0],[0,-1],[0,1],[-2,-2],[2,-2],[-2,2],[2,2],[-2,0],[2,0],[0,-2],[0,2]]:
                #     Y,X = XytoYX(*tuple(i))
                #     Y += k[0]
                #     X += k[1]
                #     if (Y,X) in path:
                #         Cons1.append([path.index((Y,X)),i])
                #         break
        # print(Cons)        

        # for i in Flush_Point_Path[indexy]:
        #     if i in path:
        #         Cons1.append([path.index(i),i])

        for i in sorted(Cons1):
            Pathisas.append(i[1] + (find_gradient(Image_Gray,*i[1]),))

                
        Pathisas.append(Flush_Point_Symbol[2])
        # print(min(Start_point_2))
    return Pathisas

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
        Thining_Image ,Flush_Point_Path,Flush_Point_Conner = thin_point_path(image,contour_use,resolution_X,resolution_Y,contours,Kernel_morp_use,draw=True)
    else:
        point_path_conner(image,contours,contour_use,draw = True)
    # fill_path_image = cv2.fillPoly(black_image, [contours[1]], color=(255,255,255))


    Pathisas = Flush_Point_to_Control_isas(Thining_Image,Flush_Point_Symbol,Flush_Point_Path,Flush_Point_Conner,resolution_X,resolution_Y,image_gray)
    
    # Pathisas = Flush_Point_to_Control_2Thin(Flush_Point_Symbol,Flush_Point_Path,Flush_Point_Path[-1],Flush_Point_Control,list_conner,resolution_X,resolution_Y,Thining_Image)
    # Pathisas = Sam_OOk(Pathisas,thres=5)
    # print("Pathisas",Pathisas)

    Pathisas = Born_To_be_Flushbot(Pathisas)
    print(Pathisas)
    # Pathisas = [(56, 330, 0), (0, 0, 255), (146, 330, 255), (213, 329, 155), (249, 317, 155), (318, 246, 155), (330, 147, 255), (331, 55, 255), (254, 54, 255), (60, 61, 155), (60, 190, 155), (101, 194, 155), (181, 195, 155)]
    Pathisas = Theta_isas(Pathisas)
    print(Pathisas)    

    return image 
    


