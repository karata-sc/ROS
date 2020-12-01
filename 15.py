import cv2
import numpy as np
import math

frame = cv2.imread('curve.png')

#lines
cv2.line(frame, (0, 240), (640, 240), (255,0,0), thickness=1, lineType=cv2.LINE_4)
cv2.line(frame, (320, 0), (320, 480), (255,0,0), thickness=1, lineType=cv2.LINE_4)

hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

#white
lower = np.array([0, 0, 0])
upper = np.array([180, 118, 87])

mask_black0 = cv2.inRange(hsv, lower, upper)

#opening-closing
kernel = np.ones((18,18),np.uint8)
mask_black = cv2.morphologyEx(mask_black0, cv2.MORPH_OPEN, kernel)
mask_black = cv2.morphologyEx(mask_black0, cv2.MORPH_CLOSE, kernel)

image,contours,hierarchy = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
for i in range(len(contours)):
    rect = cv2.minAreaRect(contours[i])
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    #diagonal
    if box[2][0]>box[0][0]:
        diagonal = box[2][0]-box[0][0]
    else:
        diagonal = box[0][0]-box[2][0]

    if diagonal<0: 
        cv2.drawContours(mask_black,[box],0,(0,0,0),-1)
        cv2.drawContours(frame,[box],0,(0,0,255),-1)


skeleton1 = cv2.ximgproc.thinning(mask_black, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)

minLineLength = 30
maxLineGap = 5
lines = cv2.HoughLinesP(skeleton1,cv2.HOUGH_PROBABILISTIC, np.pi/180, 30, minLineLength,maxLineGap)

font = cv2.FONT_HERSHEY_SIMPLEX
for x in range(0, len(lines)):
    for x1,y1,x2,y2 in lines[x]:
        cv2.line(frame,(x1,y1),(x2,y2),(0,128,0),2, cv2.LINE_AA)
        pts = np.array([[x1, y1 ], [x2 , y2]], np.int32)
        p1 = x2-x1
        p2 = y2-y1
        if p1<0:
            p1*=-1
        if p2<0:
            p2*=-1
        ans = p2/p1
        ans = math.degrees(math.atan(ans))
        if ans>30 and ans<60:
            cv2.polylines(frame, [pts], True, (0,0,255),2)
            cv2.putText(frame,"Tracks Detected", (x2,y2), font, 0.6, (0,0,255))
        else:
            cv2.polylines(frame, [pts], True, (0,255,0))

font = cv2.FONT_HERSHEY_SIMPLEX

cv2.imshow('frame',frame)
cv2.imshow('mask_black',mask_black)
cv2.imshow('skeleton1',skeleton1)
cv2.imwrite('track.png',frame)
cv2.imwrite('skeleton1.png',skeleton1)

cv2.waitKey(0)
cv2.destroyAllWindows()
