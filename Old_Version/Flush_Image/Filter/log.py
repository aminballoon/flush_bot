import math
import numpy as np
import cv2

def gammaTranform(c,gamma,image):
    h,w,d = image.shape[0],image.shape[1],image.shape[2]
    new_img = np.zeros((h,w,d),dtype=np.float32)
    for i in range(h):
        for j in range(w):
            new_img[i,j,0] = c*math.pow(image[i, j, 0], gamma)
            new_img[i,j,1] = c*math.pow(image[i, j, 1], gamma)
            new_img[i,j,2] = c*math.pow(image[i, j, 2], gamma)
    cv2.normalize(new_img,new_img,0,255,cv2.NORM_MINMAX)
    new_img = cv2.convertScaleAbs(new_img)
    return new_img

img = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\Moduel_image\task51.jpg',1)
new_img = gammaTranform(1,0.3,img)
cv2.imshow('x',new_img)
cv2.imwrite("result.png",new_img)
cv2.waitKey(0)
cv2.destroyAllWindows()


