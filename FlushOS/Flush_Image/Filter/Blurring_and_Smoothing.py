import serial
from time import sleep


def bytesy(integer):
    return divmod(integer, 0x100)

def Flush_PositionZ_Gripper(Position_Z,Orentation,Position,Time,method='bytey'):
    Data_Frame = [0xBD,0x61,*bytesy(Position_Z),Orentation,Position,*bytesy(Time)]
    CheckSum = ( ~(sum(Data_Frame[1:])) % 256 ) % 256
    Data_Frame.extend([CheckSum])
    if method.lower() == 'list':
        return Data_Frame
    if method.lower() == 'bytey':
        return bytearray(Data_Frame)


print(" ".join(str(x) for x in Flush_PositionZ_Gripper(300,0,50,7996)))