import cv2
import numpy as np
import time
capture = cv2.VideoCapture(0)  #read the video
capture.set(3,320.0) #set the size
capture.set(4,240.0)  #set the size
capture.set(5,15)  #set the frame rate
for i in range(0,2):
    flag, trash = capture.read() #starting unwanted null value
    while cv2.waitKey(1) != 27:
        flag, frame = capture.read() #read the video in frames
        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)#convert each frame to grayscale.
        blur=cv2.GaussianBlur(gray,(5,5),0)#blur the grayscale image
        ret,th1 = cv2.threshold(blur,35,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)#using threshold remave noise
        ret1,th2 = cv2.threshold(th1,127,255,cv2.THRESH_BINARY_INV)# invert the pixels of the image frame
        img,contours, hierarchy = cv2.findContours(th2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #find the contours
        cv2.drawContours(frame,contours,-1,(0,255,0),3)
        cv2.imshow('frame',frame) #show video
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        else:
            for cnt in contours:
                if cnt is not None:
                    area = cv2.contourArea(cnt)# find the area of contour
                    if area>=500 :
                        M = cv2.moments(cnt)
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        if cx<=150:
                            l=(cx*100/160)
                        elif cx>=170:
                            print("haha")
                            time.sleep(.08)
                        elif cx>151 and cx<169:
                            print("ha")
                            time.sleep(.3)
                        else:
                            print("jaja")
                            time.sleep(.08)
                    else:
                        print("ja")
                        time.sleep(.08)
                else:
                    print("gaga")
                    time.sleep(.1)