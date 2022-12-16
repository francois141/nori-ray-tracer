import os
os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"
import cv2
import math
import numpy as np
from scipy import signal
import argparse

# Parse input path
parser = argparse.ArgumentParser()
parser.add_argument('--img_path', type=str, required=True, help='Image that should be denoised. Can be with .exr')
parser.add_argument('--var_path', type=str, required=True, help='Image with pixel variance estimates. It must be png or jpg.')
args = parser.parse_args()

# Read base image
img = cv2.imread(args.img_path, cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype("float") / 255

# Read pixel variance estimates
img_variance = cv2.imread(args.var_path, cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
img_variance = cv2.cvtColor(img_variance, cv2.COLOR_BGR2GRAY).astype("float") / 255

# Base parameters for the algorithm
epsilon = 1e-3
k = 0.2
r = 3
flt = 0
wgtsum = 0
f = 3

# Implementation of d2
def d2(ngb,img,img_variance,k):
    v1 = img_variance + np.minimum(img_variance,ngb_variance)
    v2 = img_variance + ngb_variance
    sum = np.sum(np.power(ngb-img,2),axis=2) - v1
    sum /= epsilon + k*k*v2
    return sum

# Implementation of the shift function
def shift(data,dx,dy):
    data = np.roll(data,dx,0)
    data = np.roll(data,dy,1)
    return data

# Creation of a simple box filter
def boxFilter(size):
    size = 2*size+1
    return np.ones(shape=(size,size)) / pow(size,2)


if __name__ == "__main__":
    
    # Creation of the base image
    outputImage = np.zeros(shape=img.shape)

    # Make pixel variance more smooth
    #img_variance = signal.convolve2d(img_variance,boxFilter(3),mode='same')

    # Run the fast algorithm given in the slides
    for dx in range(-r,r+1):
        for dy in range(-r,r+1):
            # Keep track of progress
            print("{} : {}".format(dx,dy))
            ngb = shift(img,dx,dy)
            ngb_variance = shift(img_variance,dx,dy)
            d2pixel = d2(ngb,img,ngb_variance,k)
            d2patch = signal.convolve2d(d2pixel,boxFilter(f-1),mode='same')
            wgt = np.exp(-np.maximum(0,d2patch))
            wgt = signal.convolve2d(wgt,boxFilter(f-1),mode='same')
            outputImage += np.expand_dims(wgt,axis=2) * ngb
            wgtsum += wgt
    outputImage /= np.expand_dims(wgtsum,axis=2)

    # Display the images
    cv2.imshow("denoised_image",outputImage)
    cv2.imshow("base_image",img)

    # Wait until we are done
    cv2.waitKey(0)
    cv2.destroyAllWindows()