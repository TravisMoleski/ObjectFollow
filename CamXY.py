import numpy as np
import cv2
import imutils
import time
import datetime
from threading import Thread
import threading
from imutils.video import VideoStream
from imutils.video import FPS as fps
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import PID

P = 1.5
I = 0.0
D = 100

lowerBound=np.array([15,90,20])
upperBound=np.array([70,255,255])

mh = Adafruit_MotorHAT(addr=0x6f)

usingPiCamera = False
WaitTime = 1

W = 300
H = 300

cam = VideoStream(src = 0).start()
fps = fps().start()
kernelOpen = np.ones((5, 5))
kernelClose = np.ones((20, 20))
SetPointCONT = (0,0)

myMotorL = mh.getMotor(1)
myMotorR = mh.getMotor(2)

# turn on motor
myMotorL.run(Adafruit_MotorHAT.RELEASE);
myMotorR.run(Adafruit_MotorHAT.RELEASE);

myMotorL.setSpeed(150)
myMotorR.setSpeed(150)
myMotorL.run(Adafruit_MotorHAT.FORWARD)
myMotorR.run(Adafruit_MotorHAT.FORWARD)

def get_centroid():
    while True:
        try:
            img = cam.read()
            img = imutils.resize(img,width=W,height=H)
            width,height = img.shape[:2]
            SetX = int(height/2)
            SetY = int(width/2)
            SetPoint = (SetX,SetY)
            k = cv2.waitKey(1) % 0x100
            if k%256 == 27:
                cam.release()
                cv2.destroyAllWindows()
                print("Closing...")
                break
            # convert BGR to HSV
            imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # create the Mask
            mask = cv2.inRange(imgHSV, lowerBound, upperBound)
            # morphology
            maskOpen = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelOpen)
            maskClose = cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernelClose)
            maskFinal = maskClose
            _,conts,_ = cv2.findContours(maskFinal.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cv2.drawContours(img, conts, -1, (255, 0, 0), 3)
            for i in range(len(conts)):
                x, y, w, h = cv2.boundingRect(conts[i])
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                contours = cv2.findContours(maskFinal, mode=cv2.RETR_LIST,method=cv2.CHAIN_APPROX_SIMPLE)
                ob = contours[0]
                M = cv2.moments(ob)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                Xdev = cx/SetX-1
                Ydev = -(cy / SetY - 1)
                
                value = Xdev
                p = (PID.PID(P,I,D))               
                p.setPoint(0.0)
                #pid = (p.update(Xdev))
                pid = int(p.update(value))
                print(pid)
                
                if pid > 0:
                
                    myMotorL.setSpeed(np.absolute(pid))
                    myMotorL.run(Adafruit_MotorHAT.FORWARD);

                    myMotorR.setSpeed(100);
                    myMotorR.run(Adafruit_MotorHAT.FORWARD);
                    
                elif pid < 0:
                    
                    myMotorL.setSpeed(np.absolute(100))
                    myMotorL.run(Adafruit_MotorHAT.FORWARD);

                    myMotorR.setSpeed(np.absolute(pid))
                    myMotorR.run(Adafruit_MotorHAT.FORWARD);
                    
                else:
                    myMotorL.setSpeed(100)
                    myMotorR.setSpeed(100)
                    print("Forward")
                
                XD = str("{:.2f}".format(Xdev))
                YD = str("{:.2f}".format(Ydev))
                cv2.circle(img,(cx,cy),5,(255,255,0),2)
                cv2.circle(img,SetPoint,5,(255,255,255),2)
                cv2.putText(img,XD, (60, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                cv2.putText(img,"X=", (40, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                cv2.putText(img, YD, (60, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                cv2.putText(img,"Y=", (40, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        except KeyboardInterrupt:
            print("Shutting Down")
            myMotorL.run(Adafruit_MotorHAT.RELEASE);
            myMotorR.run(Adafruit_MotorHAT.RELEASE);
                       
        #cv2.imshow("maskClose", maskClose)
        #cv2.imshow("maskOpen", maskOpen)
        #cv2.imshow("mask", mask)
        fps.update()
        fps.stop()
        frames = ("FPS: {:.2f}".format(fps.fps()))
        cv2.putText(img,frames,(110,35), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1, cv2.LINE_AA)      
        cv2.imshow("cam", img)
        cv2.waitKey(WaitTime)

get_centroid()
