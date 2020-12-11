import serial
import cv2
from time import sleep
from math import sqrt

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
    cv2.imwrite(r"C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_main\Flush_Image\Flush_Flied_Image\Photo" + str(number)+".jpg",frame)
    cap.release()

def Flush_Reset(PIC):
    PIC.rts = 1
    sleep(4)
    PIC.rts = 0

# def Flush_PositionZ_Gripper(Position_Z,Orentation,Position,Time,method='bytey'):
#     Data_Frame = [0xBD,0x61,*bytesy(Position_Z),Orentation,Position,*bytesy(Time)]
#     CheckSum = ( ~(sum(Data_Frame[1:])) % 256 ) % 256
#     Data_Frame.extend([CheckSum])
#     if method.lower() == 'list':
#         return Data_Frame
#     if method.lower() == 'bytey':
#         return bytearray(Data_Frame)

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


# def Calculate_Trajectory_Time(Position_X,Position_Y):
#     PIC.read()
#     PIC.write(Flush_Command('callpositionxandy'))
#     PIC.read()
#     Position_From_PIC = str(PIC.readline())
#     pose_x , pose_y = (Decode_Data(Position_From_PIC))
#     Delta_x = pose_x - (Position_X*154)
#     Delta_y = pose_y - (Position_Y*154)
#     time = (sqrt((Delta_y * Delta_y) + (Delta_x * Delta_x))) / 7.0
#     return int(time)

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
        
def Targectory_Gen(x1,y1,x2,y2):
    P1 = (x1,y1)
    P2 = (x2,y2)
    delta_x = P2[0]-P1[0]
    delta_y = P2[1]-P1[1]
    r = sqrt(pow(delta_x,2)+pow(delta_y,2))
    
    Theta = atan2(delta_y,delta_x)
    Theta = int(degrees(Theta))
    if Theta < 0:
        Theta += 180 
    Theta = Theta/2
    return int(r),int(Theta), int(r*cos(Theta)) , int(r*sin(Theta))

# if __name__ == "__main__":
    # List_of_Position = [(330, 75, 0), (0, 0, 270),(230, 77, 270), (197,90, 270),(113, 240, 180), (69, 308, 170)]

    # Flush_Print_Mutil_Type(Flush_Orentation_Gripper(45,method='bytey'))
    # Flush_Print_Mutil_Type(Flush_Orentation_Gripper(30,method='bytey'))
    # Flush_Print_Mutil_Type(Flush_Orentation_Gripper(50,method='bytey'))
    # Flush_Print_Mutil_Type(Flush_Orentation_Gripper(60,method='bytey'))
    # Flush_Print_Mutil_Type(Flush_Orentation_Gripper(70,method='bytey'))
    # Flush_Print_Mutil_Type(Flush_Orentation_Gripper(90,method='bytey'))