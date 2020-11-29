import cv2
import numpy as np
from math import sqrt

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
image = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\Moduel_image\ready_field(1).jpg')
black_image = np.zeros((600, 600, 1), np.uint8)
# kuy = [[518, 187], [520, 525], [186, 525], [186, 459], [454, 448], [453, 189]]
kuy = [[132, 102], [293, 262], [263, 293], [135, 165], [126, 405], [91, 411], [89, 108]]
# kuy = [[518, 187], [520, 525]]
# duo = []
# for i in range(len(kuy)):
#     i = len(kuy) - i -1
#     for j in range(i-1,0,-1):
#         x = kuy[i][0] - kuy[j][0]
#         y = kuy[i][1] - kuy[j][1]
#         # print(x*x)
#         # print(y*y)
#         ans = int(sqrt((y*y)+(x*x)))
#         print(ans)
#         if ans in range(0,400):
#             print(kuy[i],kuy[j])
#             print(ans)
#             cv2.circle(black_image, (kuy[i][1],kuy[i][0]) , 3, (255, 255, 255), -1)
#             cv2.circle(black_image, (kuy[j][1],kuy[j][0])  , 3, (255, 255, 255), -1)
#             cv2.circle(black_image, find_middle(kuy[j][1],kuy[j][0],kuy[i][1],kuy[i][0])  , 3, (255, 255, 255), -1)

for i in range(len(kuy)):
    for j in kuy:
        x = kuy[i][0] - j[0]
        y = kuy[i][1] - j[1]
        ans = int(sqrt((y*y)+(x*x)))
        if ans in range(40,100):
            print(kuy[i],j,ans)
            cv2.circle(image, (kuy[i][0],kuy[i][1])  , 3, (255, 255, 0), -1)
            cv2.circle(image, (j[0],j[1])  , 3, (0, 255, 255), -1)
            cv2.circle(image, find_middle(j[0],j[1],kuy[i][0],kuy[i][1])  , 3, (255, 255, 255), -1)



            # cv2.circle(black_image, (187, 517)  , 3, (255, 255, 255), -1)
# cv2.circle(black_image, (187,518) , 3, (255, 255, 255), -1)
# cv2.circle(black_image, (189,453) , 3, (255, 255, 255), -1)
cv2.imshow("qweqwe",image)
cv2.waitKey(0)


# [[518, 187], [520, 525]

# print(sqrt(4+))