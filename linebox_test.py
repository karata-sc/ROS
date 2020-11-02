# -*- coding: utf-8 -*-
#-----------------------------------------
import cv2
import math
import numpy as np
import copy as cp

white_pixel = 255

frame = cv2.imread('cross.png')
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

#pick black
lower_b = np.array([0, 0, 0])                                              
upper_b = np.array([180,110,118])
mask_black = cv2.inRange(hsv, lower_b, upper_b)        

#pick_white
lower_w = np.array([0, 0, 157])
upper_w = np.array([180,43,255])
mask_white = cv2.inRange(hsv, lower_w, upper_w)

#opening-closing processing
kernel = np.ones((10,10),np.uint8)
mask_balck = cv2.morphologyEx(mask_black, cv2.MORPH_OPEN, kernel)
mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kernel)
mask_black_frame = mask_black
#mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)
#mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)


#--- initialize ---------------------
d = 0
n=0
x_cnt = 0

line_mtx = np.full((8,4,2),0)

x, y = 0,0 
h, w = 480,80 
#------------------------------------

for depth in range(8):

    cnt = 0
    mask_black2 = mask_black[y:y+h, x:x+w]

    image,contours,hierarchy = cv2.findContours(mask_black2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #----------------------------------------
    for i in range(len(contours)):
        rect = cv2.minAreaRect(contours[i])
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        #--------------------------------
        for m in range(4):
            box[m][0] += 80 * x_cnt

        #---- sort ----------------------
        org_list = np.full((4,2),0)
        for m in range(4):
            org_list[m][0] = box[m][0]
            org_list[m][1] = box[m][1]
        #---- sort_x --------------------
        sort_x = org_list[np.argsort(org_list[:, 0])]
        #---- sort_y --------------------
        sort_y = org_list[np.argsort(org_list[:, 1])]

        #---- diagonal ------------------
        diagonal_w = sort_x[3][0] - sort_x[0][0]
        diagonal_h = sort_x[3][1] - sort_x[0][1]
        diagonal_w = abs(diagonal_w)
        diagonal_h = abs(diagonal_h)
        diagonal = diagonal_w**2 + diagonal_h**2
        diagonal = int(math.sqrt(diagonal))

        #---- choose rect ---------------
        if  diagonal<100 and sort_y[0][1]-30>0 and mask_white[sort_y[0][1]-30, sort_y[0][0]] == white_pixel:
            #cv2.line(frame, (sort_x[0][0], sort_x[0][1]), (sort_x[3][0],sort_x[3][1]), (255, 255, 0))
            #cv2.circle(frame, (sort_y[0][0], sort_y[0][1]), 8, (255, 0, 0), thickness=-1)
            print diagonal
            print ""

            #cv2.circle(frame, (box[1][0]+30, box[1][1]+30), 8, (255, 0, 0), thickness=-1)

            if cnt==0:
                box1= cp.copy(sort_y)
            elif cnt==1:
                box2= cp.copy(sort_y)

            cnt += 1

        #---- choose line -------------------
    if cnt==2:
        if box1[3][1]<box2[3][1]:
            cv2.drawContours(frame,[box2],0,(0,0,255),1)
            cv2.circle(frame, (box2[2][0], box2[2][1]), 5, (0, 255, 0), thickness=-1)
            for m in range(4):
                line_mtx[depth][m][0] = box2[m][0]
                line_mtx[depth][m][1] = box2[m][1]

        elif box2[3][1]<box1[3][1]:
            cv2.drawContours(frame,[box1],0,(0,0,255),1)
            cv2.circle(frame, (box1[2][0], box1[2][1]), 5, (0, 255, 0), thickness=-1)
            for m in range(4):
                line_mtx[depth][m][0] = box1[m][0]
                line_mtx[depth][m][1] = box1[m][1]

    if cnt==1:
        cv2.drawContours(frame,[box1],0,(0,0,255),1)
        cv2.circle(frame, (box1[3][0], box1[3][1]), 5, (0, 255, 0), thickness=-1)
        for m in range(4):
            line_mtx[depth][m][0] = box1[m][0]
            line_mtx[depth][m][1] = box1[m][1]
#-----------------------------------------
    x += 80
    x_cnt += 1

print line_mtx
#--------------------------------------

cv2.imshow('pick_white',mask_white)
cv2.imshow('pickblack',mask_black2)
cv2.imshow('mask_black_frame',mask_black_frame)
cv2.imshow('frame',frame)

cv2.waitKey(0)
cv2.destroyAllWindows()
#----- END OF FILE -------------------
