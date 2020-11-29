import cv2
list_crop_img = []
image = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\Moduel_image\test_field.jpg')
blur = cv2.pyrMeanShiftFiltering(image, 11, 21)
gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_MASK)[1]
cnts,hie = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cnts = cnts[0] if len(cnts) == 2 else cnts[1]
# print(hie)
for c in cnts:
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 1 * peri, True)
    if len(approx) == 4:
        x,y,w,h = cv2.boundingRect(approx)
        if (w in range(60,100)):
            pose_x = int(x+(w/2))
            pose_y = int(y+(h/2))

            cv2.rectangle(image,(x,y),(x+w,y+h),(36,255,12),1)
            cv2.circle(image,(pose_x,pose_y),10,(0,0,255),cv2.FILLED)
            list_crop_img.append([x,x+w,y,y+h])
        # print(x,y)
offset = 4

print(list_crop_img)
for i in list_crop_img:
    cv2.imshow(str(i),image[(i[2]-offset):(i[3]+offset), (i[0]-offset):(i[1]+offset)])
    # cv2.imwrite(str(i)+".png",image[(i[2]-offset):(i[3]+offset), (i[0]-offset):(i[1]+offset)])
    cop = image[(i[2]-offset):(i[3]+offset), (i[0]-offset):(i[1]+offset)]
    im = cv2.cvtColor(cop, cv2.COLOR_BGR2GRAY)
    _,im = cv2.threshold(im, 128, 255, cv2.THRESH_BINARY)
    moments = cv2.moments(im)
    huMoments = cv2.HuMoments(moments)
    for j in range(0,7):
        huMoments[j] = -1* copysign(1.0, huMoments[j]) * log10(abs(huMoments[j]))
    print(huMoments)

# cv2.imshow('thresh', gray)
cv2.imshow('image', image)
# cv2.imshow("img",image[0:200,165:240])
# print(list_crop_img)
cv2.waitKey(0)
