import serial
import cv2
from time import sleep

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
pose_x = 400
pose_y = 400
pose_x_highbyte , pose_x_lowbyte  = bytesy(pose_x)
pose_y_highbyte , pose_y_lowbyte  = bytesy(pose_y)

data = bytearray([byte_start,byte_id,pose_x_highbyte,pose_x_lowbyte,pose_y_highbyte,pose_y_lowbyte])

checkSumOrdList = data[1:]
checkSumOrdListSum = sum(checkSumOrdList)
CheckSum = ( ~(checkSumOrdListSum) % 256 ) % 256
data.extend(bytes([CheckSum]))

print(hex(CheckSum))
print(data)
ser.write(data)
# sleep(0.2)
# while(1):
#     print(ser.readline())


ser.close()
