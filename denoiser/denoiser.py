import os
os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"

import cv2
import math
import numpy as np
from scipy import signal
import argparse

# TODO
# 1) Debug
# 2) Argument parsing
# 3) Clean code
# 4) Refaire le code bien en c++

#parser = argparse.ArgumentParser(description='Get the paths for the images')
#parser.add_argument('path', metavar='path', type=str,help='Path of the noisy image')

#args = parser.parse_args()
#print(args.path)

PATH_TO_IMAGE:str = "image.exr"
PATH_TO_IMAGE_VAR:str = "image_var.png"

img = cv2.imread(PATH_TO_IMAGE, cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

img_variance = cv2.imread(PATH_TO_IMAGE_VAR)
img_variance = cv2.cvtColor(img_variance, cv2.COLOR_BGR2GRAY)

x = img.shape[0]
y = img.shape[1]

epsilon = 1e-6
k = 0.01

r = 5
flt = 0
wgtsum = 0
f = 5

def d2(ngb,img,img_variance,k):
    v1 = img_variance + np.minimum(img_variance,ngb_variance)
    v2 = img_variance + ngb_variance
    sum = np.sum(np.power(ngb-img,2),axis=2) - v1
    sum /= epsilon + k*k*v2
    return sum

def boxFilter(size):
    size = 2*size+1
    return np.ones(shape=(size,size)) / pow(size,2)

flt = np.zeros(shape=img.shape)

def shift(data,dx,dy):
    data = np.roll(data,dx,0)
    data = np.roll(data,dy,1)
    return data

for dx in range(-r,r+1):
    for dy in range(-r,r+1):
        ngb = shift(img,dx,dy)
        ngb_variance = shift(img_variance,dx,dy)

        d2pixel = d2(ngb,img,ngb_variance,k)
        d2patch = signal.convolve2d(d2pixel,boxFilter(f-1),mode='same')
        wgt = np.exp(-np.maximum(0,d2patch))
        wgt = signal.convolve2d(wgt,boxFilter(f-1),mode='same')
        flt += np.expand_dims(wgt,axis=2) * ngb
        wgtsum += wgt

flt /= np.expand_dims(wgtsum,axis=2)

cv2.imshow("denoised_image",flt)
cv2.imshow("base_image",img)
cv2.imshow("base_image_variance",img_variance)

cv2.waitKey(0)
cv2.destroyAllWindows()