import cv2 as cv
import numpy as np
from urllib.request import urlopen
import os
import datetime
import time
import sys

#change to your ESP32-CAM ip
url="http://192.168.2.106:81/stream"
CAMERA_BUFFRER_SIZE=4096
stream=urlopen(url)
bts=b''
while True:
    bts+=stream.read(CAMERA_BUFFRER_SIZE)
    jpghead=bts.find(b'\xff\xd8')
    jpgend=bts.find(b'\xff\xd9')
    if jpghead>-1 and jpgend>-1:
        jpg=bts[jpghead:jpgend+2]
        bts=bts[jpgend+2:]
        img=cv.imdecode(np.frombuffer(jpg,dtype=np.uint8),cv.IMREAD_UNCHANGED)
        v=cv.flip(img,0)
        h=cv.flip(img,1)
        p=cv.flip(img,-1)        
        frame=p
        h,w=frame.shape[:2]
        img=cv.resize(frame,(800,600))
        ##cv.imshow("capture",img)
        
    gray_image = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    gaus =cv.adaptiveThreshold(gray_image,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY,115,1)
    params = cv.SimpleBlobDetector_Params()

    params.minThreshold = 100
    params.maxThreshold = 300
    
    params.filterByColor= 1
    params.blobColor = 255
    params.filterByArea = 1
    params.maxArea = 700000
    params.minArea = 2000
    params.filterByCircularity = 0
    params.minCircularity = 0.1
    params.filterByConvexity = 0
    params.minConvexity = 0.87
    params.filterByInertia = 0
    params.minInertiaRatio = 0.01
    ver = (cv.__version__).split('.')
    if int(ver[0]) < 3 :
        detector = cv.SimpleBlobDetector(params)
    else : 
        detector = cv.SimpleBlobDetector_create(params)
    keypoints = detector.detect(img)
    im_with_keypoints = cv.drawKeypoints(gaus, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv.imshow("Keypoints", im_with_keypoints)
    total_count = 0
    for i in keypoints:
        total_count = total_count + 1
        img_with_keypoints = cv.drawKeypoints(img, keypoints, np.array([]), (0, 0, 255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    
    k=cv.waitKey(1)
    datetime_object = datetime.now()
    print(datetime_object,total_count)
    if k & 0xFF==ord('q'):
      break
    
cv.destroyAllWindows()