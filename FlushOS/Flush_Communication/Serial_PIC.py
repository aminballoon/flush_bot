import serial
import cv2
from time import sleep


def bytesy(integer):
    return divmod(integer, 0x100)

PIC = serial.Serial(
    port = "COM10",
    baudrate = 115200,
    timeout=1
)

def Flush_Print_Mutil_Type(Data_Frame,method = "int"):
    if method.lower() == 'int':
        print (" ".join(str(x) for x in Data_Frame))
    if method.lower() == 'hex':
        print (" ".join(hex(x) for x in Data_Frame))



def Flush_Take_Photo():
    cap = cv2.VideoCapture(1+cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # turn the autofocus off
    _, frame = cap.read()
    cv2.imwrite(str(i)+".jpg",frame)
    cap.release()

def Flush_Reset(PIC):
    PIC.rts = 1
    sleep(0.2)
    PIC.rts = 0

def Flush_PositionXY(Position_X,Position_Y,method='list'):
    Data_Frame = [0xBD,0x40,*bytesy(Position_X),*bytesy(Position_Y)]
    CheckSum = ( ~(sum(Data_Frame[1:])) % 256 ) % 256
    Data_Frame.extend([CheckSum])
    if method.lower() == 'list':
        return Data_Frame
    if method.lower() == 'byte':
        return Data_Frame

def Flush_Command(method):
    Check = method.lower().replace(" ","")
    if Check == 'starttimer':
        return bytearray([0xBD,0x50,0x51,0x51])
    if Check == 'callpositionxandy':
        return bytearray([0xBD,0x20,0x21,0x21])
    if Check == 'callpositionx':
        return bytearray([0xBD,0x23,0x24,0x24])
    if Check == 'callpositiony':
        return bytearray([0xBD,0x25,0x26,0x26])
    
PIC.rts = 0
sleep(2)
byte_start = 0xBD
byte_id = 0x40

cmd = [(290,200),(300,20),(300,50),(300,100),(300,150),(300,200),(300,250),(300,300),(300,350),(300,400)
      ,(150,200),(200,200),(250,200),(350,200),(400,200)]
# PIC.write(cmd('Start Timer'))
PIC.read()
for position in cmd:
    # Flush_Print_Mutil_Type(Flush_PositionXY(*position),method = 'HEX')
    # print(Flush_Command(method = 'Call Position X and Y'))
    # PIC.write(Flush_Command(method = 'Call Position X and Y'))
    # sleep(0.5)
    # while(PIC.read() != b'\xca'):
    #     pass
    # print(PIC.read())
    PIC.write(Flush_PositionXY(*position,method='byte'))
    while(PIC.read() != b'\xac'):
        pass
        
    PIC.write(Flush_Command(method = 'Start timer'))
    while(PIC.read() != b'\xac'):
        pass
    while(PIC.read() != b'\xca'):
        pass
    sleep(1)
    print("Yes")
    # print(PIC.read())
    # PIC.write(bytearray([0xBD,0x50,0x51,0x51]))
    # sleep(0.2)
    # print(PIC.read())

    

    # sleep(2)
    # Take_Photo()
    

# print(PIC.readline())
# print(PIC.readline())
# print(PIC.readline())
# print(PIC.readline())
# print(PIC.readline())
# print(PIC.read())
# while(1):
#     print(PIC.readline())


PIC.close()
