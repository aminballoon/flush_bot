import cv2
import numpy as np

image = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\Moduel_image\Filed_img.png')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (3,3), 1)
cv2.imshow('thrasdadsesh', blur)
blur1 = cv2.GaussianBlur(gray, (0,0), 1)
cv2.imshow('thrasssssssssssssdadsesh', blur1)
thresh = cv2.threshold(blur1, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

# Remove dotted lines
cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# cnts = cnts[0] if len(cnts) == 2 else cnts[1]
# for c in cnts:
#     area = cv2.contourArea(c)
#     if area < 5000:
#         cv2.drawContours(thresh, [c], -1, (0,0,0), -1)
    # cv2.imshow('thresh', thresh)

# Fill contours
close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
close = 255 - cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, close_kernel, iterations=6)
cnts = cv2.findContours(close, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# cnts = cnts[0] if len(cnts) == 2 else cnts[1]
# for c in cnts:
#     area = cv2.contourArea(c)
#     if area < 1000:
#         cv2.drawContours(close, [c], -1, (0,0,0), -1)
#     else:
#         cv2.drawContours(close, c, -1, (0,50,0), -1)

# Smooth contours
close = 255 - close
open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20,20))
opening = cv2.morphologyEx(close, cv2.MORPH_OPEN, open_kernel, iterations=3)

# Find contours and draw result
cnts = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = cnts[0] if len(cnts) == 2 else cnts[1]
for c in cnts:
    cv2.drawContours(image, [c], -1, (36,255,12), 3)

cv2.imshow('thresh', thresh)
cv2.imshow('opening', opening)
cv2.imshow('image', image)
cv2.waitKey(0)