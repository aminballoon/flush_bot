import cv2
import numpy as np
from matplotlib import pyplot as plt
img_rgb = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\Moduel_image\test_field.jpg')
img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
template = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\377.png',0)
w, h = template.shape[::-1]
res = cv2.matchTemplate(img_gray,template,cv2.TM_CCOEFF_NORMED)
threshold = 0.8
loc = np.where( res >= threshold)
for pt in zip(*loc[::-1]):
    cv2.circle(img_rgb, (int(pt[0] + w/2), int(pt[1] + h/2)),2,(0,0,255))
    break
    # cv2.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)
cv2.imshow('res',img_rgb)
cv2.waitKey(0)




# import cv2
# import numpy as np
# img_rgb = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\HW\example.png',cv2.IMREAD_UNCHANGED)
# img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
# template = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\HW\ss.jpg',0)

# w, h = template.shape[::-1]
# res = cv2.matchTemplate(img_gray,template,cv2.TM_CCOEFF_NORMED)
# # print(res)
# threshold = 0.6
# location = np.where(res >= threshold)
# location = list(zip(*location[::-1]))
# print(location)

# if locations:
#     print('Found needle.')

#     needle_w = needle_img.shape[1]
#     needle_h = needle_img.shape[0]
#     line_color = (0, 255, 0)
#     line_type = cv.LINE_4

#     # Loop over all the locations and draw their rectangle
#     for loc in locations:
#         # Determine the box positions
#         top_left = loc
#         bottom_right = (top_left[0] + needle_w, top_left[1] + needle_h)
#         # Draw the box
#         cv.rectangle(haystack_img, top_left, bottom_right, line_color, line_type)

#     cv.imshow('Matches', haystack_img)
#     cv.waitKey()
#     #cv.imwrite('result.jpg', haystack_img)

# else:
#     print('Needle not found.')

# threshold = 0.6
# loc = np.where( res >= threshold)
# for pt in zip(*loc[::-1]):
#     cv2.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)
# cv2.imshow('res',img_rgb)
# cv2.waitKey(0)


# import numpy as np
# from PIL import Image

# im_haystack = Image.open(r"C:\Users\aminb\Desktop\FIBO\Image\Detect_something_in_picture\mario\mario.jpg")
# im_needle   = Image.open(r"C:\Users\aminb\Desktop\FIBO\Image\Detect_something_in_picture\mario\coin.jpg")

# def find_matches(haystack, needle):
#     arr_h = np.asarray(haystack)
#     arr_n = np.asarray(needle)

#     y_h, x_h = arr_h.shape[:2]
#     y_n, x_n = arr_n.shape[:2]

#     xstop = x_h - x_n + 1
#     ystop = y_h - y_n + 1

#     matches = []
#     for xmin in range(0, xstop):
#         for ymin in range(0, ystop):
#             xmax = xmin + x_n
#             ymax = ymin + y_n

#             arr_s = arr_h[ymin:ymax, xmin:xmax]     # Extract subimage
#             arr_t = (arr_s == arr_n)                # Create test matrix
#             if arr_t.all():                         # Only consider exact matches
#                 matches.append((xmin,ymin))

#     return matches

# print(find_matches(im_haystack, im_needle))