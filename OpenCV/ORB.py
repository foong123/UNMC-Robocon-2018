import numpy as np
import cv2

img2 = cv2.imread('train1.jpeg',0) # trainImage

# Initiate SIFT detector
orb = cv2.ORB_create()

# find the keypoints and descriptors with SIFT
kp2, des2 = orb.detectAndCompute(img2,None)


cap=cv2.VideoCapture(0)
while(True):
    ret,frame = cap.read()
    img1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kp1, des1 = orb.detectAndCompute(img1,None)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1,des2)
    matches = sorted(matches, key = lambda x:x.distance)
   

    img3 = cv2.drawMatches(img1,kp1,img2,kp2,matches[:20],None,flags=2)
    cv2.imshow('image',img3)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
