import numpy as np
import cv2 as cv
import glob
import time

with open("cameraMatrix.txt") as f:
    camera_matrix = np.asarray(f.read())

with open("cameraDistortion.txt") as f:
    distortion = np.asarray(f.read())
