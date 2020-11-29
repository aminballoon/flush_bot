import serial
import cv2
from time import sleep

def bytesy(integer):
    return divmod(integer, 0x100)


ser = serial.Serial(
    port = "COM21",
    baudrate = 115200,
    timeout=1
)

# ser.rts = 0
sleep(1.5)
byte_start = 0xBD
byte_id = 0x61
pose_z = 200
oren_grip = 32
pose_grip = 32
pose_z_highbyte , pose_z_lowbyte  = bytesy(pose_z)
eiei = [byte_start,byte_id,pose_z_highbyte,pose_z_lowbyte,oren_grip,pose_grip ]
data = bytearray(eiei)

checkSumOrdList = data[1:]
checkSumOrdListSum = sum(checkSumOrdList)
print(checkSumOrdListSum & 0xFF) 
CheckSum = ( ~(checkSumOrdListSum) & 0xFF ) & 0xFF
eiei.append(CheckSum)
data.extend(bytes([CheckSum]))

ser.write(data)
print(eiei)

ser.close()
