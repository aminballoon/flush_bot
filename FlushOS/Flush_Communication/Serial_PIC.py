import serial
import cv2
from time import sleep
from math import sqrt,pi,atan2


PIC = serial.Serial(
    port = "COM9",
    baudrate = 115200,
    timeout=1
)

Arduino = serial.Serial(
    port = "COM3",
    baudrate = 115200,
    timeout=1
)

def bytesy(integer):
    return divmod(integer, 0x100)
    
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

def Flush_PositionXY(Position_X,Position_Y,Position_Z=0,Orentation_Z=0,method='bytey'):
    Data_Frame = [0xBD,0x40,*bytesy(Position_X),*bytesy(Position_Y+5)]
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
    return int(float(Data)*1000) - 300


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


PIC.rts = 0

# Flush_Reset()
sleep(2)

### Gripper Position (380,340)

# List_of_Position = [
# (365,50),(365,150),(365,250),(365,350),(365,100),(365,200),(365,300),(365,400),(300,400),
# (380,50),(380,150),(380,250),(380,350),(380,100),(380,200),(380,300),(380,400),
# (400,50),(400,150),(400,250),(400,350),(400,100),(400,200),(400,300),(400,400),
# (420,50),(420,150),(420,250),(420,350),(420,100),(420,200),(420,300),(420,400)]

List_of_Position =  [(54, 335, 0, 45), (0, 0, 255, 45), (143, 334, 245, 48), (247, 323, 165, 70), (331, 218, 165, 90), (331, 147, 255, 90), 
(331, 58, 255, 46), (260, 60, 255, 44), (51, 55, 155, 1), (59, 195, 155, 43), (105, 198, 155, 45), (180, 196, 155, 45)]

# List_of_Position = [(330, 75, 270), (230, 77, 270), (197,90, 270),(113, 240, 180), (69, 308, 170)]
# List_of_Position = [(330, 75, 0), (0, 0, 270),(230, 77, 270), (197,90, 270),(113, 240, 180), (69, 308, 170)]

# List_of_Position = [(330, 72, 0), (0, 0, 270), (230, 77, 270), (196, 92, 270), (113, 240, 170), (67, 306, 170)]

# List_of_Position = [(329, 77, 0, 45), (0, 0, 255, 45), (238, 78, 255, 54), (200, 91, 255, 75), (111, 246, 155, 76), (75, 315, 155, 76)]
# List_of_Position = [(329, 188, 0, 46), (0, 0, 255, 46), (265, 190, 255, 46), (120, 192, 155, 45), (55, 193, 155, 45)]

# List_of_Position = [(247,199,80),(148,200,180),(68, 205,180)]
# PIC.write(cmd('Start Timer'))
PIC.read()
num = 0

# Start

PIC.write((Flush_PositionXY(380,335))) #Move XY to pick Gripper
while(PIC.read() != b'\xac'):
    pass
sleep(0.5)
PIC.write(Flush_Command(method = 'Start timer'))
while(PIC.read() != b'\xac'):
    pass
while(PIC.read() != b'\xca'):
    pass
sleep(1)
Arduino.write((Flush_Position_Gripper(70)))
while(Arduino.read() != b'\xac'):
    pass
while(Arduino.read() != b'\xca'): 
    pass
sleep(0.5)
Arduino.write(Flush_Orentation_Gripper(45,method='bytey'))
while(Arduino.read() != b'\xac'):
    pass
while(Arduino.read() != b'\xca'):
    pass
sleep(0.5)
Arduino.write((Flush_Position_Z(110,4000)))
while(Arduino.read() != b'\xac'):
    pass
while(Arduino.read() != b'\xca'):
    pass
sleep(0.5)
Arduino.write((Flush_Position_Gripper(25)))
while(Arduino.read() != b'\xac'):
    pass
while(Arduino.read() != b'\xca'):
    pass
sleep(0.5)
Arduino.write((Flush_Position_Z(0,4000)))
while(Arduino.read() != b'\xac'):
    pass
while(Arduino.read() != b'\xca'):
    pass

# # Gripper 
# sleep(1)

for Position in List_of_Position:
    num += 1
    if Position[1] == 0 and Position[0] == 0:
        Arduino.write(Flush_Position_Z(Position[2],2500))
        while(Arduino.read() != b'\xac'):
            pass
        while(Arduino.read() != b'\xca'):
            pass
    else:
        
        PIC.write((Flush_PositionXY(*Position)))
        while(PIC.read() != b'\xac'):
            pass

        PIC.write(Flush_Command(method = 'Call time'))
        while(PIC.read() != b'\xac'):
            pass

        Time = Decode_Data(PIC.readline())
        print(Time)
        if int(Time) <= 0:
            Time = 2

        Arduino.write(Flush_Position_Z(Position[2],Time))
        while(Arduino.read() != b'\xac'):
            pass

        PIC.write(Flush_Command(method = 'Start timer'))
        while(PIC.read() != b'\xac'):
            pass
        while(PIC.read() != b'\xca'):
            pass
        while(Arduino.read() != b'\xca'):
            pass

        Arduino.write(Flush_Orentation_Gripper(Position[3],method='bytey'))
        while(Arduino.read() != b'\xac'):
            pass
        while(Arduino.read() != b'\xca'):
            pass

    sleep(2)


# for Position in List_of_Position:
#     num += 1
#     PIC.write((Flush_PositionXY(*Position)))
#     while(PIC.read() != b'\xac'):
#         pass
#     PIC.write(Flush_Command(method = 'Start timer'))
#     while(PIC.read() != b'\xac'):
#         pass
#     while(PIC.read() != b'\xca'):
#         pass
#     if Position != (300,400):
#         Flush_Take_Photo(num)



PIC.close()
