# A program that learns Grid Autosport and learns to get faster times.
import pyvjoy,time
import math
import numpy as np
from PIL import ImageGrab
import cv2
import os

# Declaring joystick device.
j = pyvjoy.VJoyDevice(1)

file_name = 'training_data.npy'

# parameters.

throttle_value = 4800
canny_thresh1 = 110
canny_thresh2 = 300

gauss_x = 5
gauss_std = 5
show_img = False

if os.path.isfile(file_name):
    print('File exists, loading previous data')
    training_data = list(np.load(file_name))
else:
    print('File does not exist, starting fresh')
    training_data = []

def draw_lines(img, lines):
    try:
        for line in lines:
            #print('Angle')
            coords = line[0]
            cv2.line(img, (coords[0],coords[1]), (coords[2],coords[3]), [255,255,255], 2)
    except:
        pass

# Region of interest.
def roi(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    masked = cv2.bitwise_and(img, mask)
    return masked

def process_img(oringal_image):
    # Convert to greyscale.
    processed_img = cv2.cvtColor(oringal_image, cv2.COLOR_BGR2GRAY)
    # Detect edges.
    processed_img = cv2.Canny(processed_img, threshold1=canny_thresh1,threshold2=canny_thresh2)
    # Blurring.
    processed_img = cv2.GaussianBlur(processed_img, (gauss_x,gauss_x), gauss_std)

    vertices = np.array([[10,375],[10,280],[250,260], [550,260], [800,280], [800,375]])
    processed_img = roi(processed_img, [vertices])

    # edges.
    lines = cv2.HoughLinesP(processed_img, 1, np.pi/180, 160, np.array([]), 105, 2)
    
    steering_output(averageLineAngle(lines))
    
    draw_lines(processed_img, lines)
    
    return processed_img

def averageLineAngle(lines):
    totalAngle = 0
    angle = 0
    try:
        for line in lines:
            if line[0,1] > line[0,3]:
                angle = 180 - math.degrees(math.atan((line[0,1]-line[0,3])/(line[0,2]-line[0,0])))            
            elif line[0,3] > line[0,1]:
                angle = math.degrees(math.atan((line[0,3]-line[0,1])/(line[0,2]-line[0,0])))
            totalAngle += angle        
            #print('Line angle is: '+ str(angle))        
        averageAngle = totalAngle/len(lines)
        #print('Average line angle is: ' + str(averageAngle))
        return averageAngle
    except:
        pass

def steering_output(average_angle):
    try:
        #print('Average Angle: ' + str(average_angle))

        if average_angle < 10:
            print('Turning hard left')
            j.set_axis(pyvjoy.HID_USAGE_X, 0x0)

        elif average_angle < 65 and average_angle > 10:
            print('Turning slight left')
            j.set_axis(pyvjoy.HID_USAGE_X, 0x1500)

        elif average_angle > 65 and average_angle < 115:
            print('Keeping straight')
            j.set_axis(pyvjoy.HID_USAGE_X, 0x4000)
        
        elif average_angle > 115 and average_angle < 170:
            print('Turning slight right')
            j.set_axis(pyvjoy.HID_USAGE_X, 0x5500)            
        
        elif average_angle > 170:
            print('Turning hard right')
            j.set_axis(pyvjoy.HID_USAGE_X, 0x8000)
            
    except:
        pass
    j.set_axis(pyvjoy.HID_USAGE_RX, 0x4000)
    j.set_axis(pyvjoy.HID_USAGE_RY, 0x4000)
    
        
def throttle():
    j.set_axis(pyvjoy.HID_USAGE_RZ, 0x2000)
    j.set_axis(pyvjoy.HID_USAGE_Z, 0x0)
       
last_time = time.time()

while(True):
    screen = np.array(ImageGrab.grab(bbox=(8,31,808,631))) # x1,y1,x2,y2
    new_screen = process_img(screen)
    
    
    print('Loop took {} seconds'.format(time.time()-last_time))
    last_time = time.time()
    #plt.hist(new_screen.ravel(),256,[0,256]); plt.show()

    if show_img == True:
        cv2.imshow('window', new_screen)
    throttle()
    
    
    j.update
    #j.reset()
    #cv2.imshow('window',cv2.cvtColor(screen, cv2.COLOR_BGR2RGB))
    if cv2.waitKey(25) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break


