#!/usr/bin/env python2        
# -*- coding: utf-8 -*-
import cv2
import math
import copy as cp
import numpy as np

#w =0 

img = cv2.imread('videoline.png')
#x, y = 0,0
#h, w = 480,80
#for depth in range(8):
#    img_trim= img[y:y+h, x:x+w]
#    cv2.imshow('img_trim',img_trim)
#    x+=80

#program
#--- cv_initialize ---------------------
white_pixel = 255
d = 0
n=0
x_cnt = 0

line_mtx = np.full((8,4,2),0)

x, y = 0,0
h, w = 480,80

lx1=None
lx2=None
ly1=None
ly2=None

#------------------------------------
cv_frame = img
cv_frame_copy = img
#----------------
cv2.line(cv_frame_copy, (80,0), (80,480), (0, 255, 255),1)
cv2.line(cv_frame_copy, (160,0), (160,480), (0, 255, 255),1)
cv2.line(cv_frame_copy, (240,0), (240,480), (0, 255, 255),1)
cv2.line(cv_frame_copy, (320,0), (320,480), (0, 255, 255),1)
cv2.line(cv_frame_copy, (400,0), (400,480), (0, 255, 255),1)
cv2.line(cv_frame_copy, (480,0), (480,480), (0, 255, 255),1)
cv2.line(cv_frame_copy, (560,0), (560,480), (0, 255, 255),1)
cv2.line(cv_frame_copy, (640,0), (640,480), (0, 255, 255),1)

#cv2.line(cv_frame_copy, (0,240), (640,240), (255, 0, 255),2)
#---------------
hsv = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2HSV)

mask_black = cv2.inRange(hsv, np.array([0,0,0]), np.array([180,255,80]))
mask_white = cv2.inRange(hsv, np.array([0,0,106]), np.array([180,255,255]))

#----  opening-closing processing  -----------------
kernel = np.ones((10,10),np.uint8)
mask_black_frame = mask_black

#-----------------------------------------------------
for depth in range(8):
    cnt = 0
    mask_black_trim = mask_black[y:y+h, x:x+w]
    image,contours,hierarchy = cv2.findContours(mask_black_trim, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    for i in range(len(contours)):
        rect = cv2.minAreaRect(contours[i])
        box = cv2.boxPoints(rect)
        box = np.int0(box)

#--------------------------------
        for m in range(4):
            box[m][0] += 80 * x_cnt

        org_list = np.full((4,2),0)
        for m in range(4):
            org_list[m][0] = box[m][0]
            org_list[m][1] = box[m][1]

#---- sort_x --------------------
        sort_x = org_list[np.argsort(org_list[:, 0])]

#---- sort_y --------------------
        sort_y = org_list[np.argsort(org_list[:, 1])]

#---- calc diagonal ------------------
        diagonal_w = sort_x[3][0] - sort_x[0][0]
        diagonal_h = sort_x[3][1] - sort_x[0][1]
        diagonal_w = abs(diagonal_w)
        diagonal_h = abs(diagonal_h)
        diagonal = diagonal_w**2 + diagonal_h**2
        diagonal = int(math.sqrt(diagonal))

        ang = 0
        if diagonal_h!=0 and diagonal_w!=0:
            ang= float(diagonal_h) / (diagonal_w)
            ang= math.degrees(math.atan(ang))
            ang= abs(ang)

#---- choose rect ---------------
        if sort_y[0][1]-30>0 and mask_white[sort_y[0][1]-70, sort_y[0][0]]==white_pixel:

#print diagonal

            if cnt==0:
                box1= cp.copy(sort_y)
            elif cnt==1:
                box2= cp.copy(sort_y)

            cnt += 1

#---- choose line -------------------
    if cnt==2:
        if box1[3][1]<box2[3][1]:
            cv2.drawContours(cv_frame_copy,[box2],0,(255,255,0),1)
            for m in range(4):
                line_mtx[depth][m][0] = box2[m][0]
                line_mtx[depth][m][1] = box2[m][1]

        elif box2[3][1]<box1[3][1]:
            cv2.drawContours(cv_frame_copy,[box1],0,(255,255,0),1)
            for m in range(4):
                line_mtx[depth][m][0] = box1[m][0]
                line_mtx[depth][m][1] = box1[m][1]

    if cnt==1:
        cv2.drawContours(cv_frame_copy,[box1],0,(255,255,0),1)
        for m in range(4):
            line_mtx[depth][m][0] = box1[m][0]
            line_mtx[depth][m][1] = box1[m][1]


#-----------------------------------------
    x += 80
    x_cnt += 1

#print line_mtx

#------ line ----------------------------------------
ch = 0
for depth in range(8):
    if line_mtx[depth][3][0]!=0 and line_mtx[depth][3][1]!=0 and ch!=1:
        lx1 = line_mtx[depth][3][0]
        ly1 = line_mtx[depth][3][1]
        ch = 1
    elif line_mtx[depth][3][0]!=0 and line_mtx[depth][3][1]!=0 and ch==1:
        lx2 = line_mtx[depth][3][0]
        ly2 = line_mtx[depth][3][1]

if lx1!=None and lx2!=None and ly1!=None and ly2!=None:
    cv2.circle(cv_frame_copy, (lx1,ly1), 8, (0, 255, 0), thickness=-1)
    cv2.circle(cv_frame_copy, (lx2,ly2), 8, (0, 255, 0), thickness=-1)

    cv2.line(cv_frame_copy, (lx1,ly1), (lx2,ly2), (0, 0, 255),5)

#---- line ang --------------------------------------
    l_w = lx2 - lx1
    l_h = ly1 - ly2
    theta = float(l_h) / float(l_w)
    theta = math.degrees(math.atan(theta))
    cv2.putText(cv_frame_copy, 'ang = '+str(theta), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 180, 255), thickness=2)

cv2.imshow('mask_balck',mask_black)
cv2.imshow('mask_w',mask_white)
cv2.imshow('img',cv_frame_copy)
#cv2.imshow('img_trim',img_trim)
#cv2.imwrite('name.png',img)
cv2.waitKey(0)
cv2.destroyAllWindows()

