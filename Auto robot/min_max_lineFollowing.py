import cv2
import numpy as np
import imutils

error = 100
lower = np.array([0,0,135])
upper = np.array([180,62,255])

cap = cv2.VideoCapture(0)

while True:
    ret,frame = cap.read()
    centerX = frame.shape[1]/2
    bgr = cv2.inRange(frame,lower,upper)
    hsv = cv2.cvtColor(frame,cv2,COLOR_BRG2HSV)
    
    #cv2.imshow('bgr',bgr)
    
    #gray = cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)

    blurred = cv2.GaussianBlur(bgr,(5,5),0)

    thresh = cv2.threshold(blurred,150,255,cv2.THRESH_BINARY)[1]
    #cv2.imshow('thresh',thresh)    
    contour = cv2.findContours(thresh.copy(),cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)

    cv2.line(frame,(0,int(frame.shape[0]/2)),
             (int(frame.shape[1]),int(frame.shape[0]/2)),(0,0,255),2)
    cv2.line(frame,(int(frame.shape[1]/2),0),
             (int(frame.shape[1]/2),int(frame.shape[0])),(0,0,255),2)

    cv2.line(frame,(int(0 + error),0),
             (int(0 + error),int(frame.shape[0])),
             (0,255,0),2)
    cv2.line(frame,(int(frame.shape[1]- error),0),
             (int(frame.shape[1] - error),int(frame.shape[0])),
             (0,255,0),2)

    if imutils.is_cv2():
        contour = contour[0]

    else:
        contour = contour[1]
        

    if len(contour)>0:
        c = max(contour,key=cv2.contourArea)
        moment = cv2.moments(c)
        if moment["m00"] != 0:
            cX = int(moment["m10"]/moment["m00"])
            cY = int(moment["m01"]/moment["m00"])
        else:
            cX = cX

        cv2.drawContours(frame,[c], -1, (0,255,0),3)
        cv2.putText(frame,'centroid',(cX,cY),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),2)
        cv2.circle(frame,(cX,cY),10,(255,0,0),-1)

        extLeft = tuple(c[c[:,:,0].argmin()][0])
        extRight = tuple(c[c[:,:,0].argmax()][0])
        extTop = tuple(c[c[:,:,1].argmin()][0])
        extBot = tuple(c[c[:,:,1].argmax()][0])

        cv2.circle(frame,extLeft,10,(0,0,150),-1)
        cv2.circle(frame,extRight,10,(0,0,150),-1)
        cv2.circle(frame,extTop,10,(0,0,150),-1)
        cv2.circle(frame,extBot,10,(0,0,150),-1)

        #print(extLeft)
        if extLeft[0]<error and (extRight[0] > (int(frame.shape[1]- error))):
            print("turn 90")

        elif cX < (centerX + 10) and cX > (centerX - 10):
            print("a")
            WA = cX - centerX
            print(WA)
            print (cX, "On Track!",WA)

        elif cX <= centerX - 10:
            WA = cX - centerX
            print(WA)
            print (cX, "Turn left!",WA)
  
        elif cX >= centerX + 10:
            WA = cX - centerX
            print (cX, "Turn Right",WA)
                                         
    else:
        print("No line")


    cv2.imshow('frame',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows
        break
