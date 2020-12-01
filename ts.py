import cv2
import math
import numpy as np
import copy


frame = cv2.imread('200.jpg')

#calib
mtx = [[647.048823,0.000000,326.544754],[0.000000,645.959963,234.463113],[0.000000,0.000000,1.000000]]
dist = [0.026716,-0.114498,0.001072,-0.004303,0.000000]
mtx = np.array(mtx)

#print mtx

dist = np.array(dist)
h,  w = frame.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
x,y,w,h = roi
frame = dst[y:y+h, x:x+w]

hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

#mask black
lower = np.array([0, 0, 0])
#upper = np.array([180, 81, 118 ])
upper = np.array([180, 118, 118 ])
mask_black = cv2.inRange(hsv, lower, upper)

#GaussianBlur
#mask_black = cv2.GaussianBlur(mask_black, (33,33), 1)

#opening-closing
kernel = np.ones((3,3),np.uint8)
mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_OPEN, kernel)
mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kernel)

image,contours,hierarchy = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

for j in range(len(contours)):
    rect = cv2.minAreaRect(contours[j])
    box = cv2.boxPoints(rect)
    box = np.int0(box)
#    cv2.drawContours(frame,[box],0,(0,0,255),2)
    if box[2][0]>box[0][0]:
        diagonal = box[2][0]-box[0][0]
    else:
        diagonal = box[0][0]-box[2][0]
    #reject
    if diagonal>100:
        cv2.drawContours(frame,[box],0,(0,0,255),2)


        pre = copy.copy(box)
        #global_c= copy.copy(box)

        global_c = np.array([[0,0],[0,0],[0,0],[0,0]])
        print box
        #print global_c

        #ts_coordinate
        for n in range(len(box)):
            box[n][1]*=-1
        for q in range(len(box)):
            box[q][0]-=320
            box[q][1]+=240

        print '\nglobal_coordinate'
        print box
        #setting
        theta_c = 50
        focus = 645
        high_c = 190

        #calc
        print '\n'
        print 'calc'
        for i in range(len(box)):
            pre_ang = float(box[i][1])/float(focus)
            theta_r = theta_c - math.degrees(math.atan(pre_ang))
            
            #degrees?
            global_c[i][1]  = high_c / math.tan(math.radians(theta_r))
            print global_c[i][1]

            pre_ans = (global_c[i][1] ** 2)+(high_c ** 2)
            pre_ans = float(math.sqrt(pre_ans))
            pre_ans *= box[i][0]
            global_c[i][0] = float(pre_ans) / float(focus)

#cv2.putText(frame, '('+str(box[0][0])+','+str(box[0][1])+')', (pre[0][0], pre[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)
#cv2.putText(frame, '('+str(box[1][0])+','+str(box[1][1])+')', (pre[1][0], pre[1][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)
#cv2.putText(frame, '('+str(box[2][0])+','+str(box[2][1])+')', (pre[2][0], pre[2][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)
#cv2.putText(frame, '('+str(box[3][0])+','+str(box[3][1])+')', (pre[3][0], pre[3][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)

#cv2.putText(frame, '('+str(global_c[0][1])+')', (pre[0][0], pre[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)
#cv2.putText(frame, '('+str(global_c[1][1])+')', (pre[1][0], pre[1][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)
#cv2.putText(frame, '('+str(global_c[2][1])+')', (pre[2][0], pre[2][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)
#cv2.putText(frame, '('+str(global_c[3][1])+')', (pre[3][0], pre[3][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)

#put global_coordinate
cv2.putText(frame, '('+str(global_c[0][0])+','+str(global_c[0][1])+')', (pre[0][0], pre[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)
cv2.putText(frame, '('+str(global_c[1][0])+','+str(global_c[1][1])+')', (pre[1][0], pre[1][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)
cv2.putText(frame, '('+str(global_c[2][0])+','+str(global_c[2][1])+')', (pre[2][0], pre[2][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)
cv2.putText(frame, '('+str(global_c[3][0])+','+str(global_c[3][1])+')', (pre[3][0], pre[3][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)

#scale
cv2.putText(frame, '-320', (0, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), thickness=1)
cv2.putText(frame, '320', (600, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), thickness=1)
cv2.putText(frame, '-240', (320, 475), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), thickness=1)
cv2.putText(frame, '240', (320, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), thickness=1)

#cv2.putText(frame, '-160', (150, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), thickness=1)
#cv2.putText(frame, '160', (450, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), thickness=1)
#cv2.putText(frame, '-120', (320, 370), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), thickness=1)
#cv2.putText(frame, '120', (320, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), thickness=1)

#line
cv2.line(frame, (0, 240), (640, 240), (255,0,0), thickness=1, lineType=cv2.LINE_4)
cv2.line(frame, (320, 0), (320, 480), (255,0,0), thickness=1, lineType=cv2.LINE_4)

cv2.imshow('mask_black', mask_black)
cv2.imshow('frame', frame)
#cv2.imwrite('p400.jpg',frame)

cv2.waitKey(0)
cv2.destroyAllWindows()
