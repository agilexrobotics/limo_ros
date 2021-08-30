
import cv2
import numpy as np
import random
from enum import Enum
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


def DetectColor(image):
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    red_min = np.array([0, 5, 150])
    red_max = np.array([8, 255, 255])
    red_min2 = np.array([175, 5, 150])
    red_max2 = np.array([180, 255, 255])

    yellow_min = np.array([20, 5, 150])
    yellow_max = np.array([30, 255, 255])

    green_min = np.array([35, 5, 150])
    green_max = np.array([90, 255, 255])

    red_thresh = cv2.inRange(hsv_img, red_min, red_max) + \
        cv2.inRange(hsv_img, red_min2, red_max2)
    yellow_thresh = cv2.inRange(hsv_img, yellow_min, yellow_max)
    green_thresh = cv2.inRange(hsv_img, green_min, green_max)

    red_blur = cv2.medianBlur(red_thresh, 5)
    yellow_blur = cv2.medianBlur(yellow_thresh, 5)
    green_blur = cv2.medianBlur(green_thresh, 5)

    red = cv2.countNonZero(red_blur)
    yellow = cv2.countNonZero(yellow_blur)
    green = cv2.countNonZero(green_blur)

    light_color = max(red, yellow, green)

    if light_color > 60:
        if light_color == red:
            return 1
        elif light_color == yellow:
            return 2
        elif light_color == green:
            return 3
    else:
        return 0


class TLState(Enum): # traffic light state
    red = 1
    yellow = 2
    green = 3

class TLType(Enum): # traffic light type
    regular = 0
    five_lights = 1
    four_lights = 2

def ImageResize(image, height, inter= cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]
    r = height / float(h)
    dim = (int(w*r), height)
    resized = cv2.resize(image, dim, interpolation = inter)
    return resized

def DetectState(image, type):
    image = ImageResize(image, 200)
    (height, width) = image.shape[:2]
    output = image.copy()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=15, maxRadius=30)

    overallState = 0
    stateArrow = 0
    stateSolid = 0

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            if i[1] < i[2]:
                i[1] = i[2]
            
            roi = image[(i[1]-i[2]):(i[1]+i[2]), (i[0]-i[2]):(i[0]+i[2])]
            color = DetectColor(roi)
            if color > 0:
                stateSolid= color
            
        
        if type == 1:
            overallState = stateArrow + stateSolid +1
        elif type == 2:
            overallState = stateArrow +7
        else:
            overallState = stateSolid
    
    return overallState

def PlotLightResult(images):
    for i , image in enumerate(images):
        plt.subplot(1, len(images), i+1)
        img = mpimg.imread(image)
        label = TLState(DetectState(cv2.imread(image), TLType.regular.value)).name
        plt.title(label)
        plt.imshow(img)
    plt.show()

def PlotRealsenseResult():
    

if __name__ == "__main__":

    light_img_path = ["images/red.jpg", "images/yellow.png", "images/green.png"]
    random.shuffle(light_img_path)
    PlotLightResult(light_img_path)