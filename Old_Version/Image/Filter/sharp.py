import cv2
import numpy as np
image = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\messageImage_1604660857654.jpg',0)

import cv2
import numpy as np
blur=((3,3),1)
erode_=(5,5)
dilate_=(3, 3)

kuy = cv2.dilate(cv2.erode(cv2.GaussianBlur(cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\messageImage_1604660857654.jpg',0), blur[0], blur[1]), np.ones(erode_)), np.ones(dilate_))*255
cv2.imshow('imgBool_erode_dilated_blured',kuy)
# edge = cv2.Canny(kuy,40,100)
# cv2.imshow('imgBool_erode_lured',edge)
cv2.waitKey(0)
# cv2.imshow("osdsoo",image)
# image = cv2.medianBlur(image,9)
# kernel = np.array([[-1,-1,-1],
#                     [-1, 9,-1],
#                     [-1,-1,-1]])
# sharpened = cv2.filter2D(image, -1, kernel) 
# dst=cv2.fastNlMeansDenoising(sharpened, None, 10, 2, 21)


# cv2.imshow("ooo",image)
# cv2.imshow("KKK",dst)
# cv2.waitKey(0)

# gaussian = cv2.GaussianBlur(dst, (7, 7), 0)
# edged = cv2.Canny(gaussian, 10,100)

# kernel = np.ones((5,5), np.uint8)
# closing = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel)

# dilation = cv2.dilate(closing,kernel,iterations = 1)

