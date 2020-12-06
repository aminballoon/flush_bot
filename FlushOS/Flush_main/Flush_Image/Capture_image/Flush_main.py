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


if __name__ == "__main__":

    # PIC = serial.Serial(
    #     port = "COM9",
    #     baudrate = 115200,
    #     timeout=1)

    # Arduino = serial.Serial(
    #     port = "COM23",
    #     baudrate = 115200,
    #     timeout=1)

    # PIC.rts = 0
    # PIC.read()
    
    

    # Image = cv2.imread(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_main\Flush_Image\Capture_image\Image\Image_Anti_Obstacle2020_12_04_23_58_13_023566.png')
    # 
    # Image = Image[45:555, 45:555]
    # cv2.imshow("PATH_sdsdIsdasdmage",Image)
    # Image = cv2.GaussianBlur(Image , (9,9), 0)
    # _,thresh1 = cv2.threshold(cv2.cvtColor(Image, cv2.COLOR_BGR2GRAY),103,255,cv2.THRESH_BINARY)
    # cv2.imshow("PATH_sdsdImage",thresh1)
    # cv2.waitKey(0)
    
    # Delete_obstacle(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_Communication\Rail_Photo')
    Image = cv2.imread(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_main\Flush_Image\Capture_image\Image\Image_Anti_Obstacle2020_12_06_02_44_54_892045.png')
    # Flush_Taskbar(Image)
    list_symbol_template = [cv2.imread(file,0) for file in glob.glob(r'C:\Users\aminb\Documents\GitHub\flush_bot\FlushOS\Flush_main\Flush_Image\Capture_image\Image\Symbol\*.jpg')]
    PATH_Image = Flush_ImageProcessing(Image,list_symbol_template,method = "thining")
    cv2.imshow("PATH_Image",PATH_Image)
    cv2.imwrite("PATH_Image.png",PATH_Image)
    cv2.waitKey(0)