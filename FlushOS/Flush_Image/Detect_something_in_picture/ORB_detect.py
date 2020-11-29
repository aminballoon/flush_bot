# import cv2 
# import matplotlib.pyplot as plt
# # %matplotlib inline

# # read images
# img1 = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\Moduel_image\example.jpg')  
# img2 = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\symbol_module\symbol3.jpg') 

# img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
# img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

# #sift
# sift = cv2.xfeatures2d.SIFT_create(nfeatures=1000,edgeThreshold=0.5)
# # orb = cv2.ORB_create(nfeatures=50)
# keypoints_1, descriptors_1 = sift.detectAndCompute(img1,None)
# keypoints_2, descriptors_2 = sift.detectAndCompute(img2,None)

# #feature matching
# bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)

# matches = bf.match(descriptors_1,descriptors_2)
# matches = sorted(matches, key = lambda x:x.distance)

# img3 = cv2.drawMatches(img1, keypoints_1, img2, keypoints_2, matches, img2, flags=2)
# # for i in keypoints_1:
# #     print(i.pt)
# #     cv2.circle(img1,(int(i.pt[1]),int(i.pt[0])),6,(0,255,255),-1)
# #     cv2.imshow("Kuy",img1)
# #     cv2.waitKey(0)
# for i in matches:
#     print(keypoints_1[i.trainIdx].pt)

    
# plt.imshow(img3),plt.show()





# 1st import the package and check its version
import MTM
# print("MTM version: ", MTM.__version__)
print("Hello")
from MTM import matchTemplates
import cv2
import matplotlib.pyplot as plt
import numpy as np

image = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\Kuy.png')
image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
template = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\symbol_module\symbolex.png',0)
re = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\symbol_module\symbol2.png',0)
rr = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\symbol_module\symbol3.png',0)
plt.show()
# 1st format the template into a list of tuple (label, templateImage)
listTemplate = [('small', template)]

# Initialise figure
# _, axarr = plt.subplots(1,3)
# axarr[0].imshow(smallCoin, cmap="gray")

for i,angle in enumerate([90,180]):
    rotated = np.rot90(template, k=i+1) # NB: np.rotate not good here, turns into float!
    listTemplate.append( (str(angle), rotated ) )

Hits = matchTemplates(listTemplate, image_gray, N_object=1, score_threshold=0.6, method=cv2.TM_CCOEFF_NORMED, maxOverlap=0.3)

print(Hits)
point = Hits.values.tolist()[0][1]
point_plot = (int(point[0]+(point[2]/2)),int(point[1]+(point[3]/2)))

cv2.circle(image,point_plot,2,(0,255,0),-1)
cv2.imshow("Ki",image)
cv2.waitKey(0)