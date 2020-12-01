import serial
import cv2
import numpy as np
import glob
from cv2 import aruco
import matplotlib.pyplot as plt
from time import sleep

from Flush_Image import *
from Flush_Communication import *
from Flush_Image import *
from Flush_Aruco import *

if __name__ == "__main__":

    PIC = serial.Serial(
        port = "COM9",
        baudrate = 115200,
        timeout=1)

    Arduino = serial.Serial(
        port = "COM23",
        baudrate = 115200,
        timeout=1)

    PIC.rts = 0
    PIC.read()
    
    Delete_obstacle(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_Communication\Rail_Photo')
    Image = cv2.imread(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_main\Flush_Image\Capture_image\Image\Image_Anti_Obstacle.png')
    # Path_Symbol = r'C:\Users\aminb\Desktop\FIBO\Image\symbol_module\*.jpg'
    # PATH_Image = Flush_ImageProcessing(Image,Path_Symbol)
    # cv2.imshow("PATH_Image",PATH_Image)
    # cv2.waitKey(0)