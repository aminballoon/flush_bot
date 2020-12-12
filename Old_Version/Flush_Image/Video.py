import cv2
cap = cv2.VideoCapture(1+cv2.CAP_DSHOW)
cap.set(3,1600)
cap.set(4,900)
# cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # turn the autofocus off
while(True):
    ret, frame = cap.read()
    cv2.imshow('Display', frame)
    cv2.waitKey(1)


# def Decode_Data(Data):
#     Data = Data.replace("'","").replace("b","").replace('\\n','').replace('\\t',',')
#     Ans = Data.split(",")
#     return (Ans[0]),(Ans[1])
#     # print((Data))

# print(Decode_Data(str(b'30790.00\t30805.00\n')))