import serial
import cv2
from time import sleep
from math import sqrt,atan2,degrees,sin,cos

def Targectory_Gen(x1,y1,x2,y2):
    P1 = (x1,y1)
    P2 = (x2,y2)
    delta_x = P2[0]-P1[0]
    delta_y = P2[1]-P1[1]
    r = sqrt(pow(delta_x,2)+pow(delta_y,2))
    
    Theta = atan2(delta_y,delta_x)
    return int(r),int(degrees(Theta))+180, int(r*cos(Theta)) , int(r*sin(Theta))

def bytesy(integer):
    return divmod(integer, 0x100)


ser = serial.Serial(
    port = "COM9",
    baudrate = 115200,
    timeout=1
)

ser.rts = 0
sleep(1)
byte_start = 0xBD
byte_id = 0x40

P1 = (0,0)
P2 = (50,50)
pose,theta,X,Y = Targectory_Gen(*P1,*P2)
t = int(pose*154/11)
print("r =" + str(pose))
print("X =" + str(X))
print("Y =" + str(Y))
print("Theta = "+str(theta))
print("time ="+str(t))

pose_highbyte , pose_lowbyte  = bytesy(pose)
t_highbyte , t_lowbyte  = bytesy(t)
theta_highbyte , theta_lowbyte  = bytesy(theta)
data = bytearray([byte_start,byte_id,pose_highbyte,pose_lowbyte,t_highbyte,t_lowbyte,theta_highbyte,theta_lowbyte])

checkSumOrdList = data[1:]
checkSumOrdListSum = sum(checkSumOrdList)
CheckSum = ( ~(checkSumOrdListSum) % 256 ) % 256
data.extend(bytes([CheckSum]))
kuy = [byte_start,byte_id,pose_highbyte,pose_lowbyte,t_highbyte,t_lowbyte,theta_highbyte,theta_lowbyte,CheckSum]
kuy = " ".join(str(x) for x in kuy)
print(kuy)
print(data)
# ser.write(data)
sleep(0.2)
# print(ser.readline())
# print(ser.readline())
# print(ser.readline())
# print(ser.readline())
# print(ser.readline())
# print(ser.readline())
# print(ser.read())
# while(1):
#     print(ser.readline())


# ser.close()
