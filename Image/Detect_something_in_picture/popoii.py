import cv2
from math import copysign, log10
def normalize_contour(img):
    cnt, _ = cv2.findContours(img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    bounding_rect = cv2.boundingRect(cnt[0])
    img_cropped_bounding_rect = img[bounding_rect[1]:bounding_rect[1] + bounding_rect[3],
                                bounding_rect[0]:bounding_rect[0] + bounding_rect[2]]

    new_height = int((1.0 * img.shape[0])/img.shape[1] * 300)
    img_resized = cv2.resize(img_cropped_bounding_rect, (300, new_height))
    return img_resized

im = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\377.png',cv2.IMREAD_GRAYSCALE)
_,im = cv2.threshold(im, 128, 255, cv2.THRESH_BINARY)
moments = cv2.moments(normalize_contour(im))
huMoments = cv2.HuMoments(moments)
for i in range(0,7):
    huMoments[i] = -1* copysign(1.0, huMoments[i]) * log10(abs(huMoments[i]))
print(huMoments)

cv2.imshow("kkkk",normalize_contour(im)) 



im = cv2.rotate(im,rotateCode=cv2.ROTATE_90_CLOCKWISE)
_,im = cv2.threshold(im, 128, 255, cv2.THRESH_BINARY)
moments = cv2.moments(normalize_contour(im))
huMoments = cv2.HuMoments(moments)
for i in range(0,7):
    huMoments[i] = -1* copysign(1.0, huMoments[i]) * log10(abs(huMoments[i]))
print(huMoments)
cv2.imshow("kkqweqwekk",im) 


im = cv2.rotate(im,rotateCode=cv2.ROTATE_180)
_,im = cv2.threshold(im, 128, 255, cv2.THRESH_BINARY)
moments = cv2.moments(normalize_contour(im))
huMoments = cv2.HuMoments(moments)
for i in range(0,7):
    huMoments[i] = -1* copysign(1.0, huMoments[i]) * log10(abs(huMoments[i]))
print(huMoments)
cv2.imshow("kkkwwwk",im) 


im = cv2.rotate(im,rotateCode=cv2.ROTATE_90_COUNTERCLOCKWISE)
_,im = cv2.threshold(im, 128, 255, cv2.THRESH_BINARY)
moments = cv2.moments(normalize_contour(im))
huMoments = cv2.HuMoments(moments)
for i in range(0,7):
    huMoments[i] = -1* copysign(1.0, huMoments[i]) * log10(abs(huMoments[i]))
print(huMoments)
cv2.imshow("kksadskk",im) 
cv2.waitKey(0)
cv2.destroyAllWindows()
