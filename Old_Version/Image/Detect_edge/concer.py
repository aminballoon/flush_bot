import cv2
import numpy as np


img = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\8oooooo.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
gray = np.float32(gray)
dst = cv2.cornerHarris(gray,5,3,0.03)
ret, dst = cv2.threshold(dst,0.1*dst.max(),255,0)
dst = np.uint8(dst)
ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
corners = cv2.cornerSubPix(gray,np.float32(centroids),(5,5),(-1,-1),criteria)
for i in range(1, len(corners)):
    print(corners[i])
pop = cv2.ximgproc.thinning(dst)
# for i in range(1, len(corners)):
#     print(corners[i,0])
#     cv2.circle(img, (int(corners[i,0]), int(corners[i,1])), 7, (0,255,0), 2)

img[dst>0.1*dst.max()]=[0,0,255]
cv2.imshow('image', img)
cv2.waitKey(0)
cv2.destroyAllWindows
