import cv2
import matplotlib.pyplot as plt
import numpy as np
# from skimage import data
# from skimage.morphology import skeletonize, thin
# from skimage.util import invert 
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
        n = int(abs( x1 - x2 ) /2)
    else:
        n = int(abs( y1 - y2 ) /2)
    dxn = dx/n
    dyn = dy/n 
    output = []
    for i in range(n):
        output.append((int(x1+(i*dxn)),int(y1+(i*dyn))))
    return output

image = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\Moduel_image\test_field.jpg')
image1 = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\Moduel_image\test_field.jpg')
pop = image.copy()
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (5,5), 0)
# blur = cv2.GaussianBlur(gray, (101,101), -10)
# blur = cv2.threshold(blur1,0,255,cv2.THRESH_BINARY)
# highboost = cv2.subtract(blur,gray)
# cv2.imshow("Contour", highboost)
# cv2.waitKey(0)
# black_image = np.zeros((738, 778, 1),np.uint8)
# black_image = np.zeros((image1.shape[0], image1.shape[1], 1), np.uint8)

# Find Canny edges 
edge = cv2.Canny(blur,40,100) 

# bi = cv2.cvtColor(edge,cv2.THRESH_BINARY)
kernel = np.ones((3,3))
# closing = cv2.dilate(edge,kernel,iterations = 1)
dilation = cv2.dilate(edge,kernel,iterations = 1)
closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
contours, hierarchy = cv2.findContours(closing , cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 
# cv2.imshow("Contour",closing)
# cv2.waitKey(0)

# cv2.drawContours(image,contours, -1, (0, 0, 255), 3)

# filled_contour = cv2.fillPoly(black_image, conc, color=(255,255,255))
# ret3,filled_contour_binary = cv2.threshold(filled_contour,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

print("Number of Contours found = " + str(len(contours))) 
# cv2.imshow('thresh', filled_contour_binary )
# cv2.waitKey(0)
# Draw all contours 
# -1 signifies drawing all contours

list_all_point_path = []
list_contours_symbol = []
list_contours_box_symbol_inside = []
list_contours_path = []
# cv2.approxPolyDP()
for i in hierarchy.tolist()[0]:  #ADD list_contours
	Parent_check = (i[3] == 0) # check in case
	Child_check_symbol = (i[2] != -1) # Symbol
	Child_check_path = (i[2] == -1) # PATH
	if Parent_check:
		if Child_check_symbol:
			list_contours_box_symbol_inside.append(hierarchy.tolist()[0].index(i))
		if Child_check_path:
			list_contours_path.append(hierarchy.tolist()[0].index(i))
	
for j in list_contours_box_symbol_inside:
	cont = 20
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
			cv2.circle(image,i,2,(0,0,255),-1)
			list_all_point_path.append(i+(128,))
		# cv2.line(image,(center_x,center_y),(center_x + cont_x , center_y),(0,0,255),3)

	if find_color(	gray,offset_left,Y) != 255:
		for i in sampling(*(X,Y),*(offset_left , Y)):
			cv2.circle(image,i,2,(0,0,255),-1)
			list_all_point_path.append(i+(128,))
		# cv2.line(image,(center_x,center_y),(center_x - cont_x , center_y),(0,0,255),3)

	if find_color(	gray,	X,	offset_up) != 255:
		for i in sampling(*(X,Y),*(X , offset_up)):
			cv2.circle(image,i,2,(0,0,255),-1)
			list_all_point_path.append(i+(128,))
			
		# cv2.line(image,(center_x,center_y),(center_x , center_y + cont_y),(0,0,255),3)

	if find_color(	gray,	X,	offset_down) != 255:
		for i in sampling(*(X,Y),*(X , offset_down)):
			cv2.circle(image,i,2,(0,0,255),-1)
			list_all_point_path.append(i+(128,))
		# cv2.line(image,(center_x,center_y),(center_x , center_y - cont_y),(0,0,255),3)
	
	# cv2.circle(image,Centor, 3, (0,0,255), -1)
	

print(list_contours_box_symbol_inside)
# print(list_contours_symbol)
# print(list_contours_path)
for k in list_contours_path:
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
				cv2.circle(image,j,2,(0,0,255),-1)
				list_all_point_path.append(j+(find_gradient(gray,*j),))
	
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
				cv2.circle(image,j,2,(0,0,255),-1)
				list_all_point_path.append(j+(find_gradient(gray,*j),))


# cv2.imshow("kkkk",image) 
# cv2.waitKey(0)


# skeleton = skeletonize(filled_contour)
# cv2.imshow('skeleton',skeleton)
# cv2.waitKey(0)
# skeleton_lee = skeletonize(filled_contour, method='lee')
# cv2.imshow('skeleton_lee', skeleton_lee)
# cv2.waitKey(0)

# thinned = thin(filled_contour_binary)
# cv2.imwrite("000.png",filled_contour_binary)
# # display results


# fig, axes = plt.subplots(2, 2, figsize=(8, 8), sharex=True, sharey=True)
# ax = axes.ravel()

# ax[0].imshow(thinned, cmap=plt.cm.gray)
# ax[0].set_title('thinned')
# ax[0].axis('off')

# ax[1].imshow(skeleton_lee, cmap=plt.cm.gray)
# ax[1].axis('off')
# ax[1].set_title('skeleton_lee', fontsize=20)

# ax[2].imshow(skeleton, cmap=plt.cm.gray)
# ax[2].axis('off')
# ax[2].set_title('skeleton', fontsize=20)


# fig.tight_layout()
# plt.show()

# f = open("pointcloud.txt", "w")
# f.write(repr(list_all_point_path))
# f.close()

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# for i in list_all_point_path:
#     ax.scatter(i[1],i[0],i[2])
# plt.show()
# cv2.waitKey(0)

# print(list_contours_box_symbol_inside)
from math import copysign, log10
for j in list_contours_box_symbol_inside:
	# cv2.drawContours(pop,contours,i,(255,0,0),2)
	cont = 0
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
	offset_right =  X + const_x
	offset_left = X - const_x
	offset_up = Y + const_y
	offset_down = Y - const_y
	im = cv2.cvtColor(pop[offset_down:offset_up,offset_left:offset_right], cv2.COLOR_BGR2GRAY)
	cv2.imwrite(str(offset_down)+".png",pop[offset_down:offset_up,offset_left:offset_right])
	# cv2.imshow("222kkkk",pop[offset_down:offset_up,offset_left:offset_right]) 
	# cv2.waitKey(0)
	_,im = cv2.threshold(im, 128, 255, cv2.THRESH_BINARY)
	moments = cv2.moments(im)
	huMoments = cv2.HuMoments(moments)
	for q in range(0,7):
		huMoments[q] = (-1* copysign(1.0, huMoments[q]) * log10(abs(huMoments[q])))
	print(huMoments)
	cv2.imshow("kkkk",pop[offset_down:offset_up,offset_left:offset_right]) 
	cv2.waitKey(0)

cv2.destroyAllWindows() 




