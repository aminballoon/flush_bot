import serial
import cv2
from time import sleep
from math import sqrt

def bytesy(integer):
    return divmod(integer, 0x100)

PIC = serial.Serial(
    port = "COM9",
    baudrate = 115200,
    timeout=1
)

Arduino = serial.Serial(
    port = "COM23",
    baudrate = 115200,
    timeout=1
)

def Flush_Print_Mutil_Type(Data_Frame,method = "int"):
    if method.lower() == 'int':
        print (" ".join(str(x) for x in Data_Frame))
    if method.lower() == 'hex':
        print (" ".join(hex(x) for x in Data_Frame))



def Flush_Take_Photo(number):
    cap = cv2.VideoCapture(1+cv2.CAP_DSHOW)
    cap.set(3,1600)
    cap.set(4,900)
    sleep(1)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # turn the autofocus off
    _, frame = cap.read()
    cv2.imwrite(r"C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_Communication\Rail_Photo\Photo" + str(number)+".jpg",frame)
    cap.release()

def Flush_Reset(PIC):
    PIC.rts = 1
    sleep(4)
    PIC.rts = 0

def Flush_PositionZ_Gripper(Position_Z,Orentation,Position,Time,method='bytey'):
    Data_Frame = [0xBD,0x61,*bytesy(Position_Z),Orentation,Position,*bytesy(Time)]
    CheckSum = ( ~(sum(Data_Frame[1:])) % 256 ) % 256
    Data_Frame.extend([CheckSum])
    if method.lower() == 'list':
        return Data_Frame
    if method.lower() == 'bytey':
        return bytearray(Data_Frame)

def Flush_PositionXY(Position_X,Position_Y,Position_Z=0,method='bytey'):
    Data_Frame = [0xBD,0x40,*bytesy(Position_X),*bytesy(Position_Y)]
    CheckSum = ( ~(sum(Data_Frame[1:])) % 256 ) % 256
    Data_Frame.extend([CheckSum])
    if method.lower() == 'list':
        return Data_Frame
    if method.lower() == 'bytey':
        return bytearray(Data_Frame)

def Flush_PositionZ_Gripper(Position_Z,Orentation,Position,Time,method='bytey'):
    Data_Frame = [0xBD,0x61,*bytesy(Position_Z),Orentation,Position,*bytesy(Time)]
    CheckSum = ( ~(sum(Data_Frame[1:])) % 256 ) % 256
    Data_Frame.extend([CheckSum])
    if method.lower() == 'list':
        return Data_Frame
    if method.lower() == 'bytey':
        return bytearray(Data_Frame)

def Flush_Command(method):
    Check = method.lower().replace(" ","")
    if Check == 'starttimer':
        return bytearray([0xBD,0x50,0x51,0x51])
    if Check == 'callpositionxandy':
        return bytearray([0xBD,0x20,0x21,0x21])
    if Check == 'calltime':
        return bytearray([0xBD,0x23,0x24,0x24])
    if Check == 'callpositiony':
        return bytearray([0xBD,0x25,0x26,0x26])
    
    

def Decode_Data(Data):
    Data = str(Data)
    Data = Data.replace("'","").replace("b","").replace('\\n','').replace('\\t',',')
    return int(float(Data)*1000) - 200


def Calculate_Trajectory_Time(Position_X,Position_Y):
    PIC.read()
    PIC.write(Flush_Command('callpositionxandy'))
    PIC.read()
    Position_From_PIC = str(PIC.readline())
    pose_x , pose_y = (Decode_Data(Position_From_PIC))
    Delta_x = pose_x - (Position_X*154)
    Delta_y = pose_y - (Position_Y*154)
    time = (sqrt((Delta_y * Delta_y) + (Delta_x * Delta_x))) / 7.0
    return int(time)

PIC.rts = 0

# Flush_Reset()
sleep(2)

# List_of_Position = [
# (365,50),(365,150),(365,250),(365,350),(365,100),(365,200),(365,300),(365,400),(300,400),
# (380,50),(380,150),(380,250),(380,350),(380,100),(380,200),(380,300),(380,400),
# (400,50),(400,150),(400,250),(400,350),(400,100),(400,200),(400,300),(400,400),
# (420,50),(420,150),(420,250),(420,350),(420,100),(420,200),(420,300),(420,400)]

List_of_Position = [(285, 145),(191, 308),(151, 324)]
# List_of_Position = [(297, 197,100),
# (334, 204,100),
# (334, 341,100),
# (133, 341,200)]

# List_of_Position = [(247,199,80),(148,200,180),(68, 205,180)]
# PIC.write(cmd('Start Timer'))
PIC.read()
num = 0

# print(Calculate_Trajectory_Time(200,200))
for Position in List_of_Position:
    num += 1
    PIC.write((Flush_PositionXY(*Position)))
    while(PIC.read() != b'\xac'):
        pass
    PIC.write(Flush_Command(method = 'Call time'))
    while(PIC.read() != b'\xac'):
        pass
    Time = Decode_Data(PIC.readline())
    # print(Time)
    # Arduino.write(Flush_PositionZ_Gripper(Position[2],0,20,Time))
    PIC.write(Flush_Command(method = 'Start timer'))
    while(PIC.read() != b'\xac'):
        pass
    while(PIC.read() != b'\xca'):
        pass
    if Position != (300,400):
        Flush_Take_Photo(num)
    # PIC.write(Flush_Command('callpositionxandy'))
    # print(PIC.read())
    # Position_From_PIC = str(PIC.readline())

# print(Position_From_PIC)
# pose_x , pose_y = (Decode_Data(Position_From_PIC))




# PIC.write()
# for position in cmd:
    
    # Flush_Print_Mutil_Type(Flush_PositionXY(*position),method = 'HEX')
    # print(Flush_Command(method = 'Call Position X and Y'))
    # PIC.write(Flush_Command(method = 'Call Position X and Y'))
    # sleep(0.5)
    # while(PIC.read() != b'\xca'):
    #     pass
    # print(PIC.read())

    # PIC.write(Flush_PositionXY(*position,method='byte'))
    # while(PIC.read() != b'\xac'):
    #     pass
        
    # PIC.write(Flush_Command(method = 'Start timer'))
    # while(PIC.read() != b'\xac'):
    #     pass
    # while(PIC.read() != b'\xca'):
    #     pass
    # sleep(1)
    # Flush_Take_Photo(num)
    # num += 1


PIC.close()
