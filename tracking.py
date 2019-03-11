# This is a sample program for connecting to the Tello drone, getting video, flight data and control it by keyboard.
# It uses the Opencv's waitKey() method to get pressed keys which is not optimal. if you need a better implementation of keyboard reading, you can use PyGame library


import sys
import traceback
import tellopy
import av
import time
#import cv2.cv2 as cv2  # for avoidance of pylint error
import numpy
import cv2
import numpy as np
import math
from simple_pid import PID
pid_y=PID(0.08,0,0.05,setpoint=0,output_limits=(-300,300))
pid_z=PID(0.08,0,0.05,setpoint=0,output_limits=(-300,300))
def nothing(x):
    pass


def handler(event, sender, data, **args):   # to print flight data
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        xxx=1
        #print(data)


def main():
    drone = tellopy.Tello()

    xSPD = 0   # forward/backward
    ySPD = 0   # left/right
    zSPD = 0   # up/down
    zROT = 0   # yaw rotation

    try:
        drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)  # to pring flight data
        timeout = 0
        drone.connect()
        drone.wait_for_connection(60.0)
        container = av.open(drone.get_video_stream())
        print("***  Connected !")
      	"""mask = np.zeros((480,640,3), np.uint8)
        cv2.namedWindow('mask')
        cv2.createTrackbar('h1','mask',0,255,nothing)
        cv2.createTrackbar('s1','mask',0,255,nothing)
        cv2.createTrackbar('v1','mask',0,255,nothing)
        cv2.createTrackbar('h2','mask',0,255,nothing)
        cv2.createTrackbar('s2','mask',0,255,nothing)
        cv2.createTrackbar('v2','mask',0,255,nothing)"""
        # skip first 300 frames
        frame_skip = 300
        stopFlag = True
        failed_flag = False
        cy=0
        cz=0
        Kpy1 = 0.05
        Kpy2 = 0.05
        Kpy3=0.05
        Kpz = 0.05
        Kpx = 0.05
        w_old=0
        once_0=True
        once_1=True
        once_forward=False
        t_period=0
        t_current=0
        t_max=0
        w_max=0
        t0=0
        errorY=0
        errorZ=0
        t_sleep=0
        key=0
        key1=0
        t_clt=0
        counterspeed=0
        while True:
            for frame in container.decode(video=0):
                if 0 < frame_skip:
                    frame_skip = frame_skip - 1
                    #print(" ******** frame skipping :{}".format(frame_skip) )
                    continue
                frame_skip = 3  #if cpu load is high, increase this number, otherwise set it to 0

                image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                cv2.imshow('Original', image)

                key = cv2.waitKey(1)
                if (key==-1)and(key1==117)and(key!=0):#(key!=32)or(key!=13)or(key!=106)or(key!=119)or(key!=115)or(key!=100)or(key!=97)or(key!=101)or(key!=113))and (key1==117):
                    key=117
                key1=key
                #print("key is: ",key)
                if key == 27 :           # escape key
                    sys.exit(0)
                elif key == 32 :         # space key
                    drone.land()

                elif key == 13 :         # enter key
                    drone.takeoff()

                elif key == 117 :   
                    if(once_0==True):
                               drone.set_throttle(0.2)
                               time.sleep(1.5) 
                               once_0=False    # u key
                    """(hh,ww,chanels)=image.shape
                    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                    blur = cv2.GaussianBlur(hsv,(5,5),0)
                    # ********************************************
                    h1 = cv2.getTrackbarPos('h1','mask')
                    s1 = cv2.getTrackbarPos('s1','mask')
                    v1 = cv2.getTrackbarPos('v1','mask')
                    h2 = cv2.getTrackbarPos('h2','mask')
                    s2 = cv2.getTrackbarPos('s2','mask')
                    v2 = cv2.getTrackbarPos('v2','mask')
                    # define range of blue color in HSV
                    lower_blue = np.array([14,184,0])#14,133,87])#12,124,79])#66,150,0/green***red 0,161,52
                    upper_blue = np.array([47,255,255])#28,255,219])#96,255,255/green***red 8,255,132
                    # Threshold the HSV image to get only blue colors
                    mask = cv2.inRange(blur, lower_blue, upper_blue)
                    # Bitwise-AND mask and original image
                    _,contours,hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)
                    velocityX = 0
                    velocityY = 0
                    velocityZ = 0
                    # Find the biggest contour (if detected)
                    if len(contours) > 0:
                        c = max(contours, key=cv2.contourArea)
                        x,y,w,h= cv2.boundingRect(c)
                        (xx,yy),R=cv2.minEnclosingCircle(c)
                        xx=int(xx)
                        yy=int(yy)
                        x=int(x)
                        y=int(y)
                        h=int(h)
                        w=int(w)
                        R=int(R)
                        cv2.circle(image,(xx,yy),R,(255,0,0),3)
                        cv2.rectangle(image,(x,y),(x+h,y+w),(0,0,255),2)
                        area = cv2.contourArea(c)
                        M = cv2.moments(c)
                        if (M['m00'] !=0) and (area>40) :
                           cy = int(M['m10']/M['m00'])
                           cz = int(M['m01']/M['m00'])
                           failed_flag=0;
                        else :
                           cy,cz= 0,0
                           failed_flag=1;
                        if failed_flag == 0 :
                            errorY=ww/2-cy
                            errorZ=hh/2-cz
                        else:
                            errorY=0
                            errorZ=0
                        delta_w=w-w_old
                        w_old=w
                        if(((np.sqrt((errorY^2)+(errorZ^2))/R)<0.5)and (delta_w>0)and ((np.sqrt((errorY^2)+(errorZ^2))/R)>0)):
                            if (counterspeed <=5000*cv2.getTickFrequency()):
                                drone.set_pitch(0.33)
                            elif(counterspeed<=6000*cv2.getTickFrequency()):
                                drone.set_pitch(0)
                            elif(counterspeed<=11000*cv2.getTickFrequency()):
                                drone.set_pitch(-0.33)
                            elif(counterspeed<=12000*cv2.getTickFrequency()):
                                drone.set_pitch(0)
                            else:
                                counterspeed=0
                                
                            counterspeed=counterspeed+1
                                                          


                    velocityY=pid_y(errorY)
                    velocityZ=pid_z(errorZ)
                    res = cv2.bitwise_and(image,image, mask= mask)
                    xSPD=0
                    ySPD=velocityY
                    zSPD=velocityZ
                    print (str(hh)+"::"+str(ww))
                    print ("errorY: ",errorY," errorZ: ",errorZ," velocityY: ",velocityY," velocityZ: ",velocityZ," velocityX: ",velocityX)
                    #cv2.imshow('image1',image)
                    #cv2.imshow('mask',mask)
                    cv2.imshow('res',res)
                    # cv2.imshow('blur',blur)"""
                elif key == 106 :        # j key
                    zSPD = -50

                elif key == 119 :        # w key
                    xSPD = 50

                elif key == 115 :        # s key
                    xSPD = -50

                elif key == 100 :        # d key
                    ySPD = -50

                elif key == 97 :         # a key
                    ySPD = 50

                elif key == 101 :        # e key
                    zROT = -50

                elif key == 113 :        # q key
                    zROT = 50

                else:                   # no key
                    timeout +=1
                    if timeout > 3:      # if no key is pressed in a while
                        xSPD = 0   # forward/backward
                        ySPD = 0   # left/right
                        zSPD = 0   # up/dpwn
                        zROT = 0   # yaw rotation


                if key == -1 :
                    if stopFlag == True:   #to send stop only once
                        drone.forward(0)
                        drone.left(0)
                        drone.up(0)
                        drone.counter_clockwise(0)
                        stopFlag = False
                else :    # if a key is pressed, reset the timeout counter
                    stopFlag = True
                    timeout = 0
                    if xSPD >= 0:
                        drone.forward(xSPD)
                    else :
                        drone.backward(-xSPD)

                    if ySPD >= 0:
                        drone.right(ySPD)
                    else :
                        drone.left(-ySPD)

                    if zSPD >= 0:
                        drone.down(zSPD)
                    else :
                        drone.up(-zSPD)

                    if zROT >= 0:
                        drone.counter_clockwise(zROT)
                    else :
                        drone.clockwise(-zROT)







    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)
    finally:
        drone.quit()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
