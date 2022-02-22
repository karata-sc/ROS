#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import math  
import numpy as np
import cv2
import copy as cp

from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError

class LineBox(object):

    def __init__(self):
        #----  Publisher  -----------------
        self._lotate_pub = rospy.Publisher('lotate_direction', Int16, queue_size=1)
        self._black_pub = rospy.Publisher('black_image', Image, queue_size=1)
        self._white_pub = rospy.Publisher('white_image', Image, queue_size=1)
        self._frame_edit_pub = rospy.Publisher('frame_edit_image', Image, queue_size=1)

        #---- Subscriber  -----------------
        self._frame_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)

        self._bridge = CvBridge()
        self._lotate = Int16()

    def inRange_color(self, hsv, lower, upper):
        mask_frame = cv2.inRange(hsv, lower, upper)
        return (mask_frame)
        
    #---- 画像受信時の動作 ------------------------------------------------------
    def callback(self, data):
        try:
            cv_frame = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError, e:
            print e

    ####  OPENCV PROCESSING  ####################################################

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
        

        d_c= 0
        n_c=0
        x_cnt_c= 0
        y_cnt_c= 0

        line_mtx_c= np.full((8,4,2),0)

        x_c, y_c= 0,0
        h_c, w_c= 60,640

        lx1_c=None
        lx2_c=None
        ly1_c=None
        ly2_c=None

        #------------------------------------
        cv_frame_copy = cv_frame

        hsv = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2HSV)

        mask_black = cv2.inRange(hsv, np.array([0,0,0]), np.array([180,255,80]))
        mask_white = cv2.inRange(hsv, np.array([0,0,130]), np.array([180,255,255]))

        #----  opening-closing processing  -----------------
        kernel = np.ones((10,10),np.uint8)
        mask_black_frame = mask_black

        #-----------------------------------------------------
        for depth in range(8):

            #-----------Rect (Horizontal)----------------------------------------
            cnt = 0 
            mask_black_trim = mask_black[y:y+h, x:x+w]

            image,contours,hierarchy = cv2.findContours(
                    mask_black_trim, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

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
                else:
                    pass

                #---- choose rect 対角線の長さ等で選別---------------
                if diagonal<100 and sort_y[0][1]-30>0 and mask_white[sort_y[0][1]-70, sort_y[0][0]]==white_pixel and ang<10 and diagonal>20 and diagonal_h<90:
                    #print diagonal

                    if cnt==0:
                        box1= cp.copy(sort_y)
                    elif cnt==1:
                        box2= cp.copy(sort_y)
                    else:
                        pass

                    cnt += 1
                else:
                    pass

            #-----------Rect (Vertical)----------------------------------------
            cnt_c = 0
            mask_black_trim_c = mask_black[y_c:y_c+h_c, x_c:x_c+w_c]

            image,contours,hierarchy = cv2.findContours(
            mask_black_trim_c, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            for i_c in range(len(contours)):
                rect_c = cv2.minAreaRect(contours[i_c])
                box_c = cv2.boxPoints(rect_c)
                box_c = np.int0(box_c)

                #--------------------------------
                for m_c in range(4):
                    box_c[m_c][1] += 60 * y_cnt_c

                org_list_c = np.full((4,2),0)
                for m_c in range(4):
                    org_list_c[m_c][0] = box_c[m_c][0]
                    org_list_c[m_c][1] = box_c[m_c][1]

                #---- sort_x --------------------
                sort_x_c = org_list_c[np.argsort(org_list_c[:, 0])]

                #---- sort_y --------------------
                sort_y_c = org_list_c[np.argsort(org_list_c[:, 1])]

                #---- calc diagonal ------------------
                diagonal_w_c = sort_x_c[3][0] - sort_x_c[0][0]
                diagonal_h_c = sort_x_c[3][1] - sort_x_c[0][1]
                diagonal_w_c = abs(diagonal_w_c)
                diagonal_h_c = abs(diagonal_h_c)
                diagonal_c = diagonal_w_c**2 + diagonal_h_c**2
                diagonal_c = int(math.sqrt(diagonal_c))

                #---- choose rect ---------------
                if mask_white[sort_y_c[0][1], sort_y_c[0][0]-50] == white_pixel and sort_y_c[0][1]>240 and diagonal_w_c<50 and sort_y_c[0][1]>200:

                    if cnt_c==0:
                        box1_c= cp.copy(sort_y_c)
                    elif cnt_c==1:
                        box2_c= cp.copy(sort_y_c)
                    else:
                        pass

                    cnt_c += 1

                else:
                    pass


            #---- choose line (Horizontal) -------------------
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
                else:
                    pass

            elif cnt==1:
                cv2.drawContours(cv_frame_copy,[box1],0,(255,255,0),1)
                for m in range(4):
                    line_mtx[depth][m][0] = box1[m][0]
                    line_mtx[depth][m][1] = box1[m][1]
            else:
                pass

            #---- choose line (Vertical)-------------------
            if cnt_c==2:
                if box1_c[3][1]<box2_c[3][1]:
                    cv2.drawContours(cv_frame_copy,[box2_c],0,(0,255,0),1)
                    for m_c in range(4):
                        line_mtx_c[depth][m_c][0] = box2_c[m_c][0]
                        line_mtx_c[depth][m_c][1] = box2_c[m_c][1]

                elif box2_c[3][1]<box1_c[3][1]:
                    cv2.drawContours(cv_frame_copy,[box1_c],0,(0,255,0),1)
                    for m_c in range(4):
                        line_mtx_c[depth][m_c][0] = box1_c[m_c][0]
                        line_mtx_c[depth][m_c][1] = box1_c[m_c][1]
                else:
                    pass

            elif cnt_c==1:
                cv2.drawContours(cv_frame_copy,[box1_c],0,(0,255,0),1)
                for m_c in range(4):
                    line_mtx_c[depth][m_c][0] = box1_c[m_c][0]
                    line_mtx_c[depth][m_c][1] = box1_c[m_c][1]
            else:
                pass

            #----- 変数の更新  ------------------------------------
            x += 80                                                                                                   
            x_cnt += 1

            y_c += 60
            y_cnt_c += 1

        ch = 0
        ch_c = 0
        #------ GuideLine両端座標の取得 (Horizontal) ----------------------------------------
        for depth in range(8):
            if line_mtx[depth][3][0]!=0 and line_mtx[depth][3][1]!=0 and ch!=1:
                lx1 = line_mtx[depth][3][0]
                ly1 = line_mtx[depth][3][1]
                ch = 1
            elif line_mtx[depth][3][0]!=0 and line_mtx[depth][3][1]!=0 and ch==1:
                lx2 = line_mtx[depth][3][0]
                ly2 = line_mtx[depth][3][1]
            else:
                pass

        #------ GuideLine両端座標の取得 (Vertical) ----------------------------------------
        for depth in range(8):
            if line_mtx_c[depth][0][0]!=0 and line_mtx_c[depth][0][1]!=0 and ch_c!=1:
                lx1_c = line_mtx_c[depth][0][0]
                ly1_c = line_mtx_c[depth][0][1]
                ch_c = 1
            elif line_mtx_c[depth][2][0]!=0 and line_mtx_c[depth][3][1]!=0 and ch_c==1:
                lx2_c = line_mtx_c[depth][2][0]
                ly2_c = line_mtx_c[depth][2][1]
            else:
                pass

        #--- GuideLine Drawing (Horizontal) ------------------------
        if lx1!=None and lx2!=None and ly1!=None and ly2!=None:
            cv2.circle(cv_frame_copy, (lx1,ly1), 8, (0, 255, 0), thickness=-1)
            cv2.circle(cv_frame_copy, (lx2,ly2), 8, (0, 255, 0), thickness=-1)
            cv2.line(cv_frame_copy, (lx1,ly1), (lx2,ly2), (0, 0, 255),5)
            l_w = lx2 - lx1
            l_h = ly1 - ly2
            theta_hor = float(l_h) / float(l_w)
            theta_hor = math.degrees(math.atan(theta_hor))
            cv2.putText(cv_frame_copy, 'ang = '+str(theta_hor), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), thickness=2)

            #----  Data Publish  ---------------------------------------
            self._lotate.data = theta_hor
            self._lotate_pub.publish(self._lotate)
            rospy.loginfo('publish_data = %.2f' % (self._lotate.data))

        else :
            pass

        #--- GuideLine Drawing (Vertical) ------------------------
        if lx1_c!=None and lx2_c!=None and ly1_c!=None and ly2_c!=None:
            cv2.circle(cv_frame_copy, (lx1_c,ly1_c), 8, (0, 255, 0), thickness=-1)
            cv2.circle(cv_frame_copy, (lx2_c,ly2_c), 8, (0, 255, 0), thickness=-1)
            l_w_c = lx2_c - lx1_c
            l_h_c = ly1_c - ly2_c
            theta_ver = float(l_h_c) / float(l_w_c)
            theta_ver = math.degrees(math.atan(theta_ver))
            if abs(theta_ver)>20:
                cv2.line(cv_frame_copy, (lx1_c,ly1_c), (lx2_c,ly2_c), (255, 0, 0),5)
                cv2.putText(cv_frame_copy, 'ang = '+str(theta_ver), (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), thickness=2)
            else:
                pass

        else:
            pass


        #---- frame Publish --------------------------------------
        try:
            self._black_pub.publish(self._bridge.cv2_to_imgmsg(mask_black, 'mono8'))
            self._white_pub.publish(self._bridge.cv2_to_imgmsg(mask_white, 'mono8'))
            self._frame_edit_pub.publish(self._bridge.cv2_to_imgmsg(cv_frame_copy, 'bgr8'))
        except CvBridgeError, e:
            print e

if __name__ == '__main__':
    rospy.init_node('image_processing')
    color = LineBox()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
#----  END OF FILE ---------------------------
