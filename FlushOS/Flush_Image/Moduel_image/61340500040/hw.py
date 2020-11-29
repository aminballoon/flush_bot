from scipy import fftpack
import numpy as np
import cv2 
from skimage import io
from PIL import Image
import matplotlib.pylab as plt
import scipy.fftpack as fp

# How to
# Use DFT to obtain the frequency spectrum of the image.
# Block the high frequency components that are most likely responsible fro noise.
# Use IDFT to come back to the spatial domain.

im = cv2.imread(r'C:\Users\aminb\Desktop\FIBO\Image\Moduel_image\hw\car.png',0)
F1 = fftpack.fft2((im).astype(float))
F2 = fftpack.fftshift(F1)
w = im.shape[0]
h = im.shape[1]
for i in range(60, w, 135):
    for j in range(100, h, 500):
        if not (i == 330 and j == 500):
            F2[i-10:i+10, j-10:j+10] = 0

for i in range(0, w, 130):
    for j in range(200, h, 200):
        if not (i == 330 and j == 500):
            F2[max(0,i-15):min(w,i+15), max(0,j-15):min(h,j+15)] = 0

im1 = fp.ifft2(fftpack.ifftshift(F2)).real
plt.figure(figsize=(10,10))
plt.imshow(im1, cmap='gray')
plt.axis('off')
plt.show()
cv2.imwrite(r"C:\Users\aminb\Desktop\FIBO\Image\Moduel_image\hw\Output.png",im1)

