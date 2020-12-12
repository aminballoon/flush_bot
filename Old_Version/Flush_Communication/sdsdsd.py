import serial
import cv2
from time import sleep
cap = cv2.VideoCapture(1+cv2.CAP_DSHOW)
def bytesy(integer):
    return divmod(integer, 0x100)


ser = serial.Serial(
    port = "COM9",
    baudrate = 115200,
    timeout=1
)

ser.rts = 0
sleep(2)
byte_start = 0xBD
byte_id = 0x40

pose_x = 300
pose_y = 300
pose_x_highbyte , pose_x_lowbyte  = bytesy(pose_x)
pose_y_highbyte , pose_y_lowbyte  = bytesy(pose_y)

data = bytearray([byte_start,byte_id,pose_x_highbyte,pose_x_lowbyte,pose_y_highbyte,pose_y_lowbyte])

checkSumOrdList = data[1:]
checkSumOrdListSum = sum(checkSumOrdList)
CheckSum = ( ~(checkSumOrdListSum) % 256 ) % 256
data.extend(bytes([CheckSum]))

kuy = [byte_start,byte_id,pose_x_highbyte,pose_x_lowbyte,pose_y_highbyte,pose_y_lowbyte,CheckSum]
kuy = " ".join(str(x) for x in kuy)
# if pose_x == 0 and pose_y == 0:
#     ser.write(bytearray([0xBD,0x77,0x88]))
# else:
ser.write(data)
sleep(0.2)
print(ser.read())
ser.write(bytearray([0xBD,0x50,0x51,0x51]))
sleep(0.2)
print(ser.read())

while(ser.read() != b'\xca'):
    pass


pose_x = 300
pose_y = 200
pose_x_highbyte , pose_x_lowbyte  = bytesy(pose_x)
pose_y_highbyte , pose_y_lowbyte  = bytesy(pose_y)

data = bytearray([byte_start,byte_id,pose_x_highbyte,pose_x_lowbyte,pose_y_highbyte,pose_y_lowbyte])

checkSumOrdList = data[1:]
checkSumOrdListSum = sum(checkSumOrdList)
CheckSum = ( ~(checkSumOrdListSum) % 256 ) % 256
data.extend(bytes([CheckSum]))

ser.write(data)
sleep(0.2)
print(ser.read())
ser.write(bytearray([0xBD,0x50,0x51,0x51]))
sleep(0.2)
print(ser.read())
while(1):
    if ser.read() == b'\xca':
        break
    # print(ser.read())
ser.close()
# print(ser.read())
# print(ser.readline())
# print(ser.readline())
# print(ser.readline())
# print(ser.readline())
# print(ser.readline())
# print(ser.read())
# while(1):
#     print(ser.readline())


# ser.close()
