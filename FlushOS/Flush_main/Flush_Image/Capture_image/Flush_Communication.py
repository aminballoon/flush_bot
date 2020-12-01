
def bytesy(integer):
    return divmod(integer, 0x100)


def Flush_Print_Mutil_Type(Data_Frame,method = "int"):
    if method.lower() == 'int':
        print (" ".join(str(x) for x in Data_Frame))
    if method.lower() == 'hex':
        print (" ".join(hex(x) for x in Data_Frame))




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

def Flush_PositionXY(Position_X,Position_Y,method='bytey'):
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


def Flush_DriverXY(Data,PIC):
    PIC.write(Data)
    while(PIC.read() != b'\xac'):
        pass


def Flush_Call_Time_PIC(PIC):
    PIC.write(Flush_Command(method = 'Call time'))
    while(PIC.read() != b'\xac'):
        pass
    Time = Decode_Data(PIC.readline())
    return PIC.write(Flush_Command(method = 'Start timer'))

def Flush_Drive_Si_Sas(PIC):
    PIC.write(Flush_Command(method = 'Start timer'))
    while(PIC.read() != b'\xac'):
        pass
    while(PIC.read() != b'\xca'):
        pass