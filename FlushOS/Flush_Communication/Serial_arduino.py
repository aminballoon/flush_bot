import serial
import cv2
from time import sleep

def bytesy(integer):
    return divmod(integer, 0x100)


ser = serial.Serial(
    port = "COM23",
    baudrate = 115200,
    timeout=1
)

def Flush_PositionZ_Gripper(Position_Z,Orentation,Position,Time,method='bytey'):
    Data_Frame = [0xBD,0x61,*bytesy(Position_Z),Orentation,Position,*bytesy(Time)]
    CheckSum = ( ~(sum(Data_Frame[1:])) % 256 ) % 256
    Data_Frame.extend([CheckSum])
    if method.lower() == 'list':
        return Data_Frame
    if method.lower() == 'bytey':
        return bytearray(Data_Frame)

# ser.rts = 0
sleep(1.5)
byte_start = 0xBD
byte_id = 0x61
pose_z = 200
oren_grip = 0
pose_grip = 20
pose_z_highbyte , pose_z_lowbyte  = bytesy(pose_z)
eiei = [byte_start,byte_id,pose_z_highbyte,pose_z_lowbyte,oren_grip,pose_grip,*bytesy(15000)]
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
