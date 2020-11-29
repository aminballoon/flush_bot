import serial
import cv2
from time import sleep


def bytesy(integer):
    return divmod(integer, 0x100)

def PositionXY(Position_X,Position_Y):
    Data_Frame = [0xBD,0x40,*bytesy(Position_X),*bytesy(Position_Y)]
    CheckSum = ( ~(sum(Data_Frame[1:])) % 256 ) % 256
    Data_Frame.extend([CheckSum])
    return Data_Frame


def print_as_type(Data_Frame,method = "int"):
    if method.lower() == 'int':
        return " ".join(str(x) for x in Data_Frame)
    if method.lower() == 'hex':
        return " ".join(hex(x) for x in Data_Frame)

print(print_as_type(PositionXY(400,100),method = 'HEX'))

