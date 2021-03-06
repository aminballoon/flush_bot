import serial
import cv2
import numpy as np
import glob
from cv2 import aruco
import matplotlib.pyplot as plt
from time import sleep

from Flush_Image import *
from Flush_Image_Taskbar import *
from Flush_Communication import *
from Flush_Image import *
from Flush_Aruco import *

# Image = cv2.imread(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_main\Flush_Image\Capture_image\Image\Image_Anti_Obstacle2020_12_04_23_58_13_023566.png')

# Image = Image[45:555, 45:555]
# cv2.imshow("PATH_sdsdIsdasdmage",Image)
# Image = cv2.GaussianBlur(Image , (9,9), 0)
# _,thresh1 = cv2.threshold(cv2.cvtColor(Image, cv2.COLOR_BGR2GRAY),103,255,cv2.THRESH_BINARY)
# cv2.imshow("PATH_sdsdImage",thresh1)
# cv2.waitKey(0)

def Flush_Photo_Club():
        List_of_Position_Camera = [
        (365,50),(365,150),(365,250),(365,350),(365,100),(365,200),(365,300),(365,400),(300,400),
        (380,50),(380,150),(380,250),(380,350),(380,100),(380,200),(380,300),(380,400),
        (400,50),(400,150),(400,250),(400,350),(400,100),(400,200),(400,300),(400,400),
        (420,50),(420,150),(420,250),(420,350),(420,100),(420,200),(420,300),(420,400)]
        num = 0
        for Position in List_of_Position_Camera:
            num += 1
            PIC.write((Flush_PositionXY(*Position)))
            while(PIC.read() != b'\xac'):
                pass
            PIC.write(Flush_Command(method = 'Start timer'))
            while(PIC.read() != b'\xac'):
                pass
            while(PIC.read() != b'\xca'):
                pass
            if Position != (300,400):
                Flush_Take_Photo(num)

def Flush_GOTO_Hell():
    
    # PIC.write((Flush_PositionXY(380,335)))
    PIC.write((Flush_PositionXY(380,200))) #Move XY to pick Gripper
    while(PIC.read() != b'\xac'):
        pass
    sleep(0.5)
    PIC.write(Flush_Command(method = 'Start timer'))
    while(PIC.read() != b'\xac'):
        pass
    while(PIC.read() != b'\xca'):
        pass
    sleep(0.5)
    Arduino.write((Flush_Position_Gripper(70)))
    while(Arduino.read() != b'\xac'):
        pass
    while(Arduino.read() != b'\xca'): 
        pass
    sleep(0.2)
    Arduino.write(Flush_Orentation_Gripper(45,method='bytey'))
    while(Arduino.read() != b'\xac'):
        pass
    while(Arduino.read() != b'\xca'):
        pass
    sleep(0.2)
    Arduino.write((Flush_Position_Z(105,4000)))
    while(Arduino.read() != b'\xac'):
        pass
    while(Arduino.read() != b'\xca'):
        pass
    sleep(0.2)
    Arduino.write((Flush_Position_Gripper(20)))
    while(Arduino.read() != b'\xac'):
        pass
    while(Arduino.read() != b'\xca'):
        pass
    sleep(0.2)
    Arduino.write((Flush_Position_Z(0,4000)))
    while(Arduino.read() != b'\xac'):
        pass
    while(Arduino.read() != b'\xca'):
        pass

def Flush_GOTO_Abyss(List_of_Position):
    num = 0
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

if __name__ == "__main__":
    # Delete_obstacle(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_main\Flush_Image\Flush_Take_Photo')
    Image = cv2.imread(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_main\Flush_Image\Flush_Flied_Image\Image_Anti_Obstacle2020_12_14_10_05_08_343625.png')
    Flush_Taskbar(Image)
    list_symbol_template = [cv2.imread(file,0) for file in glob.glob(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_main\Flush_Image\Flush_Symbol\*.jpg')]
    PATH_Image = Flush_ImageProcessing(Image,list_symbol_template,method = "thining")
    cv2.imshow("PATH_Image",PATH_Image)
    cv2.imwrite("PATH_Image.png",PATH_Image)
    cv2.waitKey(0)

    # PIC = serial.Serial(
    #     port = "COM9",
    #     baudrate = 115200,
    #     timeout=1)

    # Arduino = serial.Serial(
    #     port = "COM22",
    #     baudrate = 115200,
    #     timeout=1)
    
    # PIC.rts = 0
    # PIC.read()
    # sleep(1)

    # Flush_Photo_Club()

    # Flush_GOTO_Hell()
    # List_of_Position = [(327, 58, 0, 46), (0, 0, 255, 46), (235, 56, 255, 45), (68, 59, 155, 3), (55, 237, 155, 90), (55, 329, 155, 45), (136, 329, 155, 45), (302, 326, 255, 79), (282, 279, 255, 67), (228, 227, 255, 67), (170, 171, 255, 67)]
    # Flush_GOTO_Abyss(List_of_Position)

    
    
    


