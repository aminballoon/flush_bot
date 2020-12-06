import serial
import cv2
from time import sleep

def bytesy(integer):
    return divmod(integer, 0x100)


ser = serial.Serial(
    port = "COM3",
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

def Flush_Position_Z(Position_Z,Time,method='bytey'):
    Data_Frame = [0xBD,0x61,*bytesy(Position_Z),*bytesy(Time)]
    CheckSum = ( ~(sum(Data_Frame[1:])) % 256 ) % 256
    Data_Frame.extend([CheckSum])
    if method.lower() == 'list':
        return Data_Frame
    if method.lower() == 'bytey':
        return bytearray(Data_Frame)

def Flush_Position_Gripper(Position,method='bytey'):
    Data_Frame = [0xBD,0x62,Position,0,0,0]
    CheckSum = ( ~(sum(Data_Frame[1:])) % 256 ) % 256
    Data_Frame.extend([CheckSum])
    if method.lower() == 'list':
        return Data_Frame
    if method.lower() == 'bytey':
        return bytearray(Data_Frame)

def Flush_Orentation_Gripper(Orentation,method='bytey'):
    Data_Frame = [0xBD,0x63,Orentation,0,0,0]
    CheckSum = ( ~(sum(Data_Frame[1:])) % 256 ) % 256
    Data_Frame.extend([CheckSum])
    if method.lower() == 'list':
        return Data_Frame
    if method.lower() == 'bytey':
        return bytearray(Data_Frame)
# ser.rts = 0
sleep(3)
# byte_start = 0xBD
# byte_id = 0x61
# pose_z = 5
# oren_grip = 0
# pose_grip = 0
# pose_z_highbyte , pose_z_lowbyte  = bytesy(pose_z)
# eiei = [byte_start,byte_id,pose_z_highbyte,pose_z_lowbyte,oren_grip,pose_grip,*bytesy(15000)]
# data = bytearray(eiei)

# checkSumOrdList = data[1:]
# checkSumOrdListSum = sum(checkSumOrdList)

# print(checkSumOrdListSum & 0xFF) 
# CheckSum = ( ~(checkSumOrdListSum) & 0xFF ) & 0xFF
# eiei.append(CheckSum)
# data.extend(bytes([CheckSum]))
#
# cmd = [(0,10,70,0),(110,10,70,4000),(110,10,30,0),(110,10,30,0),(0,10,30,4000)]
# for i in cmd:
    # ser.write(Flush_PositionZ_Gripper(*i))
    # print(Flush_PositionZ_Gripper(*i,method='list'))
    # while(ser.read() != b'\xac'):
        # pass
    # while(ser.read() != b'\xca'):
    #     pass
    # sleep(4)
# print(eiei)

# ser.write(Flush_Position_Gripper(70))
# print(Flush_Position_Z(200,4000,'list'))
# print(ser.read())
# print(ser.read())
# sleep(3)
# ser.write(Flush_Position_Gripper(60))
# sleep(3)
# ser.write(Flush_Position_Gripper(50))
# sleep(3)
# ser.write(Flush_Position_Gripper(40))
# sleep(3)
# ser.write(Flush_Position_Gripper(30))
# sleep(3)

ser.write(Flush_Position_Z(180,4000))
sleep(3)
ser.write(Flush_Position_Gripper(25))
sleep(3)


# print((50,'list'))
# ser.write(Flush_Position_Gripper(50))
# print(ser.read())
# print(ser.read())
# while(ser.read() != b'\xac'):
#         pass
# while(ser.read() != b'\xca'):
#         pass

# ser.write(Flush_Orentation_Gripper(10))
# while(ser.read() != b'\xac'):
#         pass
# while(ser.read() != b'\xca'):
#         pass

# ser.write(Flush_Position_Z(110,4000))
# while(ser.read() != b'\xac'):
#         pass
# while(ser.read() != b'\xca'):
#         pass

# ser.write(Flush_Position_Gripper(30))
# while(ser.read() != b'\xac'):
#         pass
# while(ser.read() != b'\xca'):
#         pass

# ser.write(Flush_Position_Z(0,4000))
# while(ser.read() != b'\xac'):
#         pass
# while(ser.read() != b'\xca'):
#         pass

ser.close()
