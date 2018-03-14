from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

# Reported Bug: https://github.com/opencv/opencv/issues/6081
cv2.ocl.setUseOpenCL(False)
# Reported Bug: https://github.com/BVLC/caffe/issues/581
CV_LOAD_IMAGE_COLOR = 1

# ---Constants, Tunable params---
# subsample ratio is used to downsize the camera image.
subsamplingRatio = 0.5
# used to remove false positive noise in the brute force match.
matchesThreshold = 0.5

# Initiate ORB detector and BruteForce Matcher
orb = cv2.ORB_create()
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Initialize Camera Capture, 0 will take the default pc camera

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)

# Read Template Image
imgSourceBGR = cv2.imread('train.jpeg', CV_LOAD_IMAGE_COLOR)
imgSourceGRAY = cv2.cvtColor(imgSourceBGR, cv2.COLOR_BGR2GRAY)

# find the key points with ORB for the Template Image
kpSource = orb.detect(imgSourceGRAY, None)
kpSource, desTemplate = orb.compute(imgSourceGRAY, kpSource)

# Sizes for camera Image
#h1 = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)*subsamplingRatio)
#w1 = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH)*subsamplingRatio)
h1 = 640
w1 = 480
# Sizes for the Source Image
h2, w2 = imgSourceBGR.shape[:2]
# Size of the result image: camera and sources images side by sie
nWidth = int(w1+w2)
nHeight = int(max(h1, h2))
hdif = int(abs(h2-h1)/2)
result = np.zeros((nHeight, nWidth, 3), np.uint8)
print('Cancel the camera capture by pressing \'q\'')
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # Capture frame-by-frame
    cameraFrameColor = frame.array
   
    cv2.cvtColor(cameraFrameColor,cv2.COLOR_BGR2GRAY)
	# show the frame
    cv2.imshow("Frame", cameraFrameColor)
    key = cv2.waitKey(1) & 0xFF

	# clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # Pre-process the Camera frame, resize and get gray Scale
    cameraFrameColor = cv2.resize(cameraFrameColor, (0, 0), fx=subsamplingRatio, fy=subsamplingRatio)
    cameraFrameGray = cv2.cvtColor(cameraFrameColor, cv2.COLOR_BGR2GRAY)

    # find the key points with ORB For the Camera Image
    kpCam = orb.detect(cameraFrameGray, None)
    kpCam, desCam = orb.compute(cameraFrameGray, kpCam)

    # find the Matches between both set of points.
    matches = bf.match(desCam, desTemplate)
    # Filter the matches
    dist = [m.distance for m in matches]
    thres_dist = (sum(dist) / len(dist)) * matchesThreshold
    matches = [m for m in matches if m.distance < thres_dist]

    # Paint the images side to side
    result[hdif:hdif+h2, :w2] = imgSourceBGR
    result[:h1, w2:w1+w2] = cameraFrameColor

    for i in range(len(matches)):
        pt_a = (int(kpSource[matches[i].trainIdx].pt[0]), int(kpSource[matches[i].trainIdx].pt[1]+hdif))
        pt_b = (int(kpCam[matches[i].queryIdx].pt[0]+w2), int(kpCam[matches[i].queryIdx].pt[1]))
        cv2.line(result, pt_a, pt_b, (255, 0, 0))

    # Display the resulting frame
    cv2.imshow('Result', result)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
camera.release()
cv2.destroyAllWindows()
