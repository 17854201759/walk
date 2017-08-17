# -*- coding: cp936 -*-
#GitHub
from __future__ import division
import os
import string
import sys
import cv
import cv2
import numpy
import math
import time
from naoqi import ALProxy

class naoBoundary:
    
    def __init__(self,ip="127.0.0.1",port=9559):
        self.imageHeader = cv.CreateImageHeader((640,480),1,3)#ͼƬ��ʽ
        self.autonomousLife = ALProxy("ALAutonomousLife",ip,port)
        self.autonomousLife.setState("disabled")#�ر���������ģʽ
        self.postureInit = ALProxy("ALRobotPosture",ip,port)
        self.postureInit.goToPosture("StandInit",0.8)#��ʼ������
        self.motion = ALProxy("ALMotion",ip,port)
        setHead=["HeadPitch","HeadYaw"]
        times = [[0.2]]*2
        self.motion.angleInterpolation(setHead,[20*math.pi/180,0],times,True)
        self.video = ALProxy("ALVideoDevice",ip,port)
        self.kCameraSelectID = 18
        cameraId = 1
        self.video.setParam(self.kCameraSelectID,cameraId)#ѡ��������ͷ
        self.subscriberID = "subscriberID"
        self.fps = 20
        self.subscriberID = self.video.subscribe(self.subscriberID, 2, 13, self.fps)#�����¼�
        
    def __del__(self):
        self.video.unsubscribe(self.subscribeID)

    def getCaptureImage(self):
        results = self.video.getImageRemote(self.subscriberID)#��ȡ����ͼ����д���
        ByteArray = bytearray(results[6])#������������ֽ�����
        nuArray = numpy.array(ByteArray) #ת����numpy����
        bgrImage = nuArray.reshape(640,480,3)#��һά�ı����ά�Ĳ�ɫͼ������Ƕ�λ��Ϊ�Ҷ�ͼ
        cv.SetData(self.imageHeader,bgrImage,0)
        ipltemp = cv.CloneImage(self.imageHeader) #������ͼ���������������ͷ��ROI�����ݡ�ipltemp����
        temp = self.imageHeader[:]#tempΪcvmat����
        return numpy.asarray(temp)#cvmat��array 

    def checkArea(self, area, max_sizearea, min_sizearea):
        if area >= max_sizearea:
            return False
        if area <= min_sizearea:
            return False
        return True
    
    def nothing(x):  
        pass
    
    def getContours(self,hsvImage):   
        cv2.namedWindow('image')
        cv2.createTrackbar('max_size', 'image', 153600, 307200, self.nothing)
        cv2.createTrackbar('min_size', 'image', 100, 5000, self.nothing)
        cv2.createTrackbar('h_min','image',0,255,self.nothing)  
        cv2.createTrackbar('h_max','image',180,255,self.nothing)  
        cv2.createTrackbar('s_min','image',0,255,self.nothing)  
        cv2.createTrackbar('s_max','image',30,255,self.nothing)  
        cv2.createTrackbar('v_min','image',221,255,self.nothing)  
        cv2.createTrackbar('v_max','image',255,255,self.nothing)      
        # HSV�Ļ�������  
        h_min=cv2.getTrackbarPos('h_min','image')  
        h_max=cv2.getTrackbarPos('h_max','image')  
        s_min=cv2.getTrackbarPos('s_min','image')  
        s_max=cv2.getTrackbarPos('s_max','image')  
        v_min=cv2.getTrackbarPos('v_min','image')  
        v_max=cv2.getTrackbarPos('v_max','image')
        # �趨��ɫ����ֵ  
        lower_white=numpy.array([h_min,s_min,v_min])  
        upper_white=numpy.array([h_max,s_max,v_max])  
        # ������ֵ������ģ ������Ҫ�궨��ɫֵ��Χ    
        mask=cv2.inRange(hsvImage,lower_white,upper_white)   
        # cv2.imshow('mask',mask)  
        # ���θ�ʴ������
            
        kernel = numpy.ones((5,5),numpy.uint8)  
        mask = cv2.erode(mask,kernel,iterations = 1)  
        mask = cv2.dilate(mask,kernel,iterations = 1)  
        mask = cv2.erode(mask,kernel,iterations = 1)  
        mask = cv2.dilate(mask,kernel,iterations = 1)
            
        # ������  
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  
        #mask_img, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)  
        contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        # ��ԭͼ�����ģ����λ����  
        res=cv2.bitwise_and(hsvImage,hsvImage,mask=mask)
        contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)  
        #contours ���������ÿ���������ǵ�ļ���
        print "�������� "
        print len(contours)
        return contours

    def getRectangleSlope(self,contours,naoImage):
        max_sizearea = cv2.getTrackbarPos('max_size', 'image')
        min_sizearea = cv2.getTrackbarPos('min_size', 'image')
        self.rectangleSlop = [0]
        if len(contours) == 0:
            self.rectangleSlop = [0]
        else:
            self.rectangleSlope = list(range(len(contours)*2))
        rectangleNumber = 0
        for white1_i in contours:            
           # white1_area = cv2.contourArea(white1_i)#��ȡ�������
            white1_x, white1_y, white1_w, white1_h = cv2.boundingRect(white1_i)
            white1_area = white1_w  *  white1_h
            if white1_y > naoImage.shape[0]*0.7:
                 continue
            elif (white1_w - white1_x)> naoImage.shape[1]*0.9:
                 self.rectangleSlope[0] = "flag"
                 break
            elif self.checkArea(white1_area, max_sizearea, min_sizearea) == False:
                 continue
            cv2.rectangle(naoImage, (white1_x, white1_y), (white1_x + white1_w, white1_y + white1_h),(0,0, 255), 2)
            self.rectangleSlope[len(contours)+rectangleNumber] = (white1_x + white1_w)#һ�����ζ�Ӧһ��x�᳤��
            if  white1_x < naoImage.shape[1]/2:
                self.rectangleSlope[rectangleNumber] = white1_h / white1_w
            else:
                self.rectangleSlope[rectangleNumber] = (white1_h / white1_w)
            rectangleNumber = rectangleNumber+1
            
        # ��ʾͼ��  
        cv2.imshow('naoImage',naoImage) 
        print "����б��"
        print self.rectangleSlope
        return self.rectangleSlope
        #cv2.waitKey(0)
        
    def getLines(self):
        self.im=cv.LoadImage('naoImage.jpg', cv.CV_LOAD_IMAGE_GRAYSCALE)
        pi = math.pi #Pi��ֵ        
        dst = cv.CreateImage(cv.GetSize(self.im), 8, 1)#����ͷ���������ݣ������ʼ���ռ��ڵ����ݣ���ͨ��
        cv.Canny(self.im, dst, 200, 200)#һ�ֱ�Ե����㷨
        cv.Threshold(dst, dst, 100, 255, cv.CV_THRESH_BINARY)#��ֵ��
         
        color_dst_standard = cv.CreateImage(cv.GetSize(self.im), 8, 3)#3ͨ����ɫͼ
        cv.CvtColor(self.im, color_dst_standard, cv.CV_GRAY2BGR)#Create output image in RGB to put red lines


        #lines = cv2.HoughLines(mask,1,np.pi/180,80)
        lines = cv.HoughLines2(dst, cv.CreateMemStorage(0), cv.CV_HOUGH_STANDARD, 1, pi / 180, 150, 0, 0)
        #im2 140
        
        for line in lines[:100]:
            print line
        return lines
    
    def getRhoThera(self,lines):
        rhoThera = [[0]*2]*len(lines)
        row = 0
        for line in lines[:100]:
            print line
            if (line[1] > 1/3*math.pi) and (line[1] < 2/3*math.pi):
                rhoThera[row][0] = line[0]
                rhoThera[row][1] = line[1]
                row=row + 1
        print "row"
        print row
        for (rho, theta) in lines[:100]:#���ǰ100��ֱ��
            a = math.cos(theta) #Calculate orientation in order to print them
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (cv.Round(x0 + 1000*(-b)), cv.Round(y0 + 1000*(a)))
            pt2 = (cv.Round(x0 - 1000*(-b)), cv.Round(y0 - 1000*(a)))
            cv.Line(self.im, pt1, pt2, cv.CV_RGB(0, 55, 0), 2, 4) #Draw the line
            cv.ShowImage("naoImageDrawLines",self.im)
        return rhoThera
    def getLastDistance(self,rhoThera):
        print "rhoThera"
        print rhoThera
        if rhoThera[0][1] < math.pi / 2:
            rho = 480-math.sin(rhoThera[0][1])*rhoThera[0][0]
        else:
            rho = 480 - math.sin(rhoThera[0][1]-math.pi/2)*rhoThera[0][0]
        rhoToDistance = rho /480 *0.5+0.05
        return rhoToDistance
class move:
    def __init__(self,ip="127.0.0.1",port=9559):
        self.motion = ALProxy("ALMotion",ip,port)
            #��̬����
        maxstepx = 0.04
        maxstepy = 0.14
        maxsteptheta = 0.4
        maxstepfrequency = 0.5
        stepheight = 0.02
        torsowx = 0.0
        torsowy = 0.0
        self.moveConfig = [["MaxStepX",maxstepx],
                ["MaxStepY",maxstepy],
                ["MaxStepTheta",maxsteptheta],
                ["MaxStepFrequency",maxstepfrequency],
                ["StepHeight",stepheight],
                ["TorsoWx",torsowx],
                ["TorsoWy",torsowy]]
        
    def changeAngle(self,deg): 
        self.motion.move(0.1, 0.0, deg*math.pi / 180,self.moveConfig)
        #time.sleep(1.0)
    def moveToo(self,x,y):
        self.motion.move(x,y,0,self.moveConfig)
        
    def moveTooEnd(self,x,y):
        #self.motion.post.moveTo(x,y,0)
        self.motion.move(x,y,0,self.moveConfig)
        time.sleep(3.0)
        self.motion.move(0,0,0)
        pass

    def killMoving(self):
        self.motion.killMove()        
if __name__=="__main__":
    ip="192.168.95.2"
    exit = False
    x = 0.5
    y = 0
    naoBoundary = naoBoundary(ip)
    naoMove = move(ip)
    naoMove.moveToo(x,y)
    while(1):
        start = time.time()
        naoImage = naoBoundary.getCaptureImage()
        
        resizeImage = cv2.resize(naoImage, None, fx=1, fy=1, interpolation=cv2.INTER_CUBIC)#ͼ��ߴ�任
        hsvImage = cv2.cvtColor(resizeImage, cv2.COLOR_BGR2HSV)#��ɫ�ռ�ת��
        contours = naoBoundary.getContours(hsvImage)
        rectangleSlope = naoBoundary.getRectangleSlope(contours,naoImage)
        print "naoImage.shape[0]*0.9 , rectangleSlope[len(contours)]"
        if len(contours) == 0:
            naoMove.moveToo(0.5,0)
            print "û��ʶ�𵽱߽�ֱ��"
        elif (rectangleSlope[0] != "flag") :
            #˵��û�е������50cm,Ҳ�������
            print "˵��û�е������50cm,Ҳ�������"
            if  abs(rectangleSlope[0]) < 1.04:
                print "��ת"
                deg = -20
                naoMove.changeAngle(deg)
            elif abs(rectangleSlope[0]) > 2.09 :
                print "��ת"
                deg = 20
                naoMove.changeAngle(deg)
            else:
                print "�������ֱ��"
                naoMove.moveToo(0.5,0)
                
        elif(rectangleSlope[0] == "flag"):
            #ֻ��һ��������X�᳤�ȴ���0.9��ͼ
            print "ʹ��Hough�任"
            #�����ڴ���ͼƬ��ͬʱ������ǰ�ߣ�����
            cv2.imwrite("naoImage.jpg",naoImage)
            lines = naoBoundary.getLines()
            rhoThera = naoBoundary.getRhoThera(lines)
            #���������ƺ�ġ�����
            if len(rhoThera) == 0:
                continue
            lastDiatance = naoBoundary.getLastDistance(rhoThera)#Ԥ�����ľ���
            
            print "lastDiatance"
            print lastDiatance
            if rhoThera[0][1] > 1.04 and rhoThera[0][1] < 1.57:
                print "��ת"
                deg = 10
                naoMove.changeAngle(deg)
                naoMove.moveTooEnd(lastDiatance,0)
                exit = True
            elif  rhoThera[0][1] >1.57 and rhoThera[0][1] < 2.09:
                print "��ת"
                deg = -10
                naoMove.changeAngle(deg)
                naoMove.moveTooEnd(lastDiatance,0)
                exit = True
            else:
                print "�������ֱ��"
                naoMove.moveTooEnd(lastDiatance,0)
                exit = True
        else:
            print ""
            naoMove.moveToo(0.5,0)
        
        print "time:"
        print time.time() - start
        key = cv2.waitKey(10)
        if key == 27:
            break
        if exit == True:
            break
        print "running"
        
    cv2.destroyAllWindows() 
