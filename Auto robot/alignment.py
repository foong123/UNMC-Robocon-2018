import cv2
import imutils
import numpy as np

error= 40

cap = cv2.VideoCapture(0)
#frame = cv2.imread('star.jpg',0)

while True:
    ret, frame = cap.read()

    #print(frame.shape)

    mask = np.zeros(frame.shape[:2],np.uint8)
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    cv2.line(frame,(0,int(frame.shape[0]/2)),(int(frame.shape[1]),int(frame.shape[0]/2)),(0,0,255),2)
    cv2.line(frame,(int(frame.shape[1]/2),0),(int(frame.shape[1]/2),int(frame.shape[0])),(0,0,255),2)
    #frame.shape[0]/2 center of y axis
    #[0] y-axis

    #error line
    #y-axis
    cv2.line(frame,(0,int(frame.shape[0]/2 + error)),(int(frame.shape[1]),int(frame.shape[0]/2 +error)),(0,255,0),2)
    cv2.line(frame,(0,int(frame.shape[0]/2 -error)),(int(frame.shape[1]),int(frame.shape[0]/2 -error)),(0,255,0),2)

    #x-axis
    cv2.line(frame,(int(frame.shape[1]/2 + error),0),(int(frame.shape[1]/2 + error),int(frame.shape[0])),(0,255,0),2)
    cv2.line(frame,(int(frame.shape[1]/2 - error),0),(int(frame.shape[1]/2 - error),int(frame.shape[0])),(0,255,0),2)
    
    
    
    blurred = cv2.GaussianBlur(gray,(5,5),0)
    
    thresh = cv2.threshold(blurred,100,255,cv2.THRESH_BINARY_INV)[1]
    
    #cv2.imshow('blurred',blurred)
    #cv2.imshow('thresh',thresh)
    
    contour = cv2.findContours(thresh.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    #contour = cv2.findContours(thresh.copy(),cv2.RETR_TREE,
    #                           cv2.CHAIN_APPROX_SIMPLE)

    
    
    if imutils.is_cv2():
        contour = contour[0]

    else:
        contour = contour[1]
            

    #if len(contour)>0:
    for c in contour:
        
        peri = cv2.arcLength(c,True)
        approx = cv2.approxPolyDP(c,0.04*peri,True)
        
        #c = max(contour,key=cv2.contourArea)
        if len(approx)==10: #change to number of edge of the target
            moment = cv2.moments(c)
            cX = int(moment["m10"]/moment["m00"])
            cY = int(moment["m01"]/moment["m00"])

            cv2.drawContours(mask,[c], -1, (255,255,255),cv2.FILLED)
            cv2.putText(frame,'centroid',(cX,cY),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),2)
            cv2.circle(frame,(cX,cY),10,(255,0,0),-1)

            if (cX - frame.shape[1]/2 < error and frame.shape[1]/2 - cX < error ) and (cY - frame.shape[0]/2 < error and frame.shape[0]/2 - cY < error):
                print("center")
                cv2.putText(frame,'center',(10,10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),2)
            #cv2.imshow('mask',mask)
    cv2.imshow('frame',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
