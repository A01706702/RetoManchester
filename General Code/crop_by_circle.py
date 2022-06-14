import cv2
import numpy as np
image_orig = cv2.imread('No_speed_limit.jpeg')
image = cv2.resize(image_orig, (300,150))
image_to_crop = cv2.resize(image_orig, (300,150))
image_to_crop = image_to_crop[25:200, 100:300]
img = image[25:200, 100:300]
image_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

############ Threshold ###############
threshold_value = 120
max_val = 255
#threshold invertido
ret, image_final = cv2.threshold(image_gray, threshold_value, max_val, cv2.THRESH_BINARY_INV)

############## circulos ############
circles = cv2.HoughCircles(image_gray,cv2.HOUGH_GRADIENT,1,20,
                           param1=50,param2=30,minRadius=13,maxRadius=60)
circles = np.uint16(np.around(circles))
for i in circles[0,:]:
    cx=i[0]
    cy=i[1]
    center=(cx,cy)
    #circle center
    cv2.circle(img,center,1,(0,0,255),2)
    #circle outline
    radius=i[2] #radius
    cv2.circle(img,center,radius,(255,0,255),1)
    #crop without circle
    cropleft = cx - radius - 8
    cropright = cx + radius + 8
    cropup = cy - radius - 8
    cropdown = cy + radius + 8
    cropped_img = image_to_crop[cropup:cropdown, cropleft:cropright] # img to show
    
cv2.imshow('To crop',image_to_crop)
cv2.imshow('detected circles',img)
cv2.imshow('Cropped Image', cropped_img)
#cv2.imshow('InverseBinaryThresholding', image_final)

cv2.waitKey(0)
cv2.destroyAllWindows()
