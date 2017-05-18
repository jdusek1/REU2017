77#!/usr/bin/env python
# -*- coding: utf-8 -*-
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
from flask import Flask, render_template, Response
from pythonds.basic.stack import Stack

from scipy import ndimage
from scipy.ndimage import morphology as scmorph

import logging
import time
import numpy as np
import cv2
import pyrealsense as pyrs
import atexit
import thread

# set up flask app
app = Flask(__name__)

# set up logging level
logging.basicConfig(level=logging.INFO)

# create a default motor object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# auto disable motors on shutdown
def turnOffMotors():
        mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

# Implement four wheel drive motor control, for maneouvering.
# Need to handle transitions properly - nonholonomic process! Use command filter.

# Eventually can close the loop - estimate current speed from optic flow OR just
# buy some encoders for the axles (cheap ones on Pololu)

# For proper control, should use a diff-drive model to convert
# velocity commands into wheel control

# should set up a dictionary or new class to handle motor attributes, like speed

# set filter coefficients (currently first order filter)
c0 = 1
c1 = 0.5
thresh = 20
wait_time = 0.5

# upon initialising, set state variables to zero. Simple version = speed control only
cs_m1 = 0
cs_m2 = 0
cs_m3 = 0
cs_m4 = 0

def simple_lowpass(v_current, v_des):
    v_new = int(round(c0*v_current + c1*(v_des - v_current)))
    # watch out for negative numbers...
    return v_new


def forwardDrive(speed_con):
    # open loop control: all wheels forward

    # can probably farm some of this out to subfunctions
    global cs_m1, cs_m2, cs_m3, cs_m4
    cs_d1 = speed_con - cs_m1
    cs_d2 = speed_con - cs_m2
    cs_d3 = speed_con - cs_m3
    cs_d4 = speed_con - cs_m4

    # set up speed lists, motor lists
    motorspeeds = [cs_m1, cs_m2, cs_m3, cs_m4]
    speed_diffs =  [cs_d1, cs_d2, cs_d3, cs_d4]

    # build up speed simultaneously until within threshold

    while (cs_d1 or cs_d2 or cs_d3 or cs_d4):

        for wheeli in range(1, 5):
            # check speed difference
            if abs(speed_diffs[wheeli-1]) < thresh:
                v_new = speed_con

            else:
                v_new = simple_lowpass(motorspeeds[wheeli-1], speed_con)

            mot = mh.getMotor(wheeli)
            if v_new < 0:
                mot.setSpeed(-1*v_new)
                mot.run(Adafruit_MotorHAT.BACKWARD)
            else:
                mot.setSpeed(v_new)
                mot.run(Adafruit_MotorHAT.FORWARD)

            # update current motor speed
            motorspeeds[wheeli-1] = v_new
            speed_diffs[wheeli-1] = speed_con - v_new
            #print(wheeli)
            #print(v_new)


        # pause to make sure motors have time to get up to new speed
        time.sleep(wait_time)

        # update current motor speeds
        # put this rubbish into a subfunction
        cs_m1 = motorspeeds[0]
        cs_m2 = motorspeeds[1]
        cs_m3 = motorspeeds[2]
        cs_m4 = motorspeeds[3]

        cs_d1 = speed_diffs[0]
        cs_d2 = speed_diffs[1]
        cs_d3 = speed_diffs[2]
        cs_d4 = speed_diffs[3]


def backwardDrive(speed_con):
    # open loop control: all wheels backward
    # currently using +/- to differentiate direction
    speed_con = speed_con*-1

    global cs_m1, cs_m2, cs_m3, cs_m4
    cs_d1 = speed_con - cs_m1
    cs_d2 = speed_con - cs_m2
    cs_d3 = speed_con - cs_m3
    cs_d4 = speed_con - cs_m4

    # set up speed lists, motor lists
    motorspeeds = [cs_m1, cs_m2, cs_m3, cs_m4]
    speed_diffs = [cs_d1, cs_d2, cs_d3, cs_d4]

    # build up speed simultaneously until within threshold

    while (cs_d1 or cs_d2 or cs_d3 or cs_d4):

        for wheeli in range(1, 5):
            # check speed difference
            # note that commanded speed is signed for direction, but motor takes unsigned values.
            # we will have to implement a check.

            if abs(speed_diffs[wheeli - 1]) < thresh:
                v_new = speed_con

            else:
                v_new = simple_lowpass(motorspeeds[wheeli - 1], speed_con)

            mot = mh.getMotor(wheeli)

            if v_new < 0:
                mot.setSpeed(-1*v_new)
                mot.run(Adafruit_MotorHAT.BACKWARD)
            else:
                mot.setSpeed(v_new)
                mot.run(Adafruit_MotorHAT.FORWARD)

            # update current motor speed
            motorspeeds[wheeli - 1] = v_new
            speed_diffs[wheeli - 1] = speed_con - v_new
            #print(wheeli)
            #print(v_new)

        # pause to make sure motors have time to get up to new speed
        time.sleep(wait_time)
        # update current motor speeds
        cs_m1 = motorspeeds[0]
        cs_m2 = motorspeeds[1]
        cs_m3 = motorspeeds[2]
        cs_m4 = motorspeeds[3]

        cs_d1 = speed_diffs[0]
        cs_d2 = speed_diffs[1]
        cs_d3 = speed_diffs[2]
        cs_d4 = speed_diffs[3]



#def leftTurn(degrees):
    # turns 'on spot': left side backwards, right side forwards

    # first stop the robot
    # odometry estimation: currently have zero feedback
    # need to close the loop for this to actually work well, but for now we will try heuristic values


#def rightTurn():
    # turns 'on spot': right side backwards, left side forwards
    # similar problems



# fancier maneouvers can be developed by decoupling front and rear motors

def shutdown_server(environ):
    if not 'werkzeug.server.shutdown' in environ:
        raise RuntimeError('Not running the development server')
    environ['werkzeug.server.shutdown']()


@app.route('/')
def index():
    return render_template('./index.html')


def gen(py_dev):
    running = True
    global depth_stack
    iLowH, iLowS, iLowV = [0,0,0]

    iHighH, iHighS, iHighV = [179,255, 255]

    while running:
        py_dev.wait_for_frame()
        c_im = py_dev.colour
        rgb_im = c_im[...,::-1]
        # try to scale with minimal wrap over effective range
        # Note that most colormaps give darker values to closer items
        # we may want to invert this, intuitively
        d_im = py_dev.depth*0.05

        # close holes without removing segmentation by doing it before converting to image
        d_im_filt = scmorph.grey_closing(d_im, size=(7,7))

        # filter out 'background' (aka depth out of range).
        d_im_col = cv2.applyColorMap(d_im_filt.astype(np.uint8), cv2.COLORMAP_HOT)
        d_im_gray = cv2.cvtColor(d_im_col, cv2.COLOR_BGR2GRAY)
        d_thresh = cv2.adaptiveThreshold(d_im_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 3,5)
        # set background to zero
        d_thresh[d_im_gray < 5] = 0

        # erode to remove white noise
        # could also  dilate to remove holes (this may desegment the image though)
        kernel = np.ones((2, 2), np.uint8)
        opening = cv2.morphologyEx(d_thresh, cv2.MORPH_OPEN, kernel, iterations=2)

        # need to isolate regions, so distance transform to ensure they're separated
        dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
        # threshold back to binary
        _, sure_fg = cv2.threshold(dist_transform,0.12*dist_transform.max(),255,0)
        sure_fg = np.uint8(sure_fg)

        _, contours, hierarchy = cv2.findContours( sure_fg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # if intensity too subtle, could use hsv as base image, compare hue?
        mean, stdev = [0,0]
        d_im_show = d_im_gray

        for cntr in contours:
            if 1500<cv2.contourArea(cntr):
                cv2.drawContours(d_im_col,[cntr],0,(0,255,0),2)
                mask = np.zeros(d_im_col.shape, dtype="uint8")
                # problem with masking
                cv2.drawContours(mask, [cntr], 0,(255,255,255), -1)
                # extract channel

                mean = cv2.mean(d_im_gray, mask)
                # get centroid
                M = cv2.moments(cntr)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.circle(d_im_col, (cX, cY), 7, (255, 255, 255), -1)
                cv2.putText(d_im_col, str(mean), (cX - 20, cY - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                print(mean)

                d_im_show = mask
                break


        # use greyscale intensity within contour regions?
       

        # label regions (eg connectedComponents)
        # label background with 0 to avoid inclusion in watershed
        #unknown = cv2.subtract(d_thresh,sure_fg)
        # Marker labelling
        #_, markers = cv2.connectedComponents(sure_fg)

        # Add one to all labels so that sure background is not 0, but 1
        #markers = markers+1

        # Now, mark the region of unknown with zero
        #markers[unknown==255] = 0

        # Apply watershed algorithm
        #d_im_hsv = cv2.applyColorMap(d_im_filt.astype(np.uint8), cv2.COLORMAP_HSV)
        #markers = cv2.watershed(d_im_hsv,markers)
        # markers now stores color-based region information.
        # find COM of brightest region (ie furthest distance) to obtain 1D directional vector
        #M = cv2.moments(markers)
        #cX = int(M["m10"] / M["m00"])
        #cY = int(M["m01"] / M["m00"])

        # output image
        #d_im_hsv[markers == -1] = [0,0,0]
        #cv2.circle(d_im_hsv, (cX, cY), 7, (255, 255, 255), -1)

        # we can filter to reduce noise but this makes things run very slow
        # so should use a much lower framerate in this case
        # may not be quickest method - see later

        #edge = cv2.Canny(blurred, 100, 255, apertureSize=5)
        #ret, otsu = cv2.threshold(d_im_gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        # binary image in otsu
        #d_tf = cv2.distanceTransform(otsu, cv2.DIST_L2, 3)
        #d_tf = cv2.normalize(d_tf, 0, 1, cv2.NORM_MINMAX)
        #_, contours, hierarchy = cv2.findContours( otsu, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #d_im_col[edge != 0] = (0, 255, 0)

        #for cntr in contours:
        #    #if hierarchy[cntr][3] >= 0: # problem here
        #    if 200<cv2.contourArea(cntr):
        #     cv2.drawContours(d_im_col,[cntr],0,(0,255,0),2)

        cd = d_im_show #np.concatenate((dist_transform, d_im_show), axis=1)
        #cd = np.concatenate((rgb_im, d_im_col), axis=1)
        ret, frame = cv2.imencode('.jpg', cd)
        jpeg_encode = frame.tobytes()
        yield(b'--frame\r\n'
              b'Content-Type: image/jpeg\r\n\r\n' + jpeg_encode + b'\r\n\r\n')



@app.route('/video_feed')
def video_feed():
    return Response(gen(pyrs.Device()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def threaded_stream():
    app.run(host='0.0.0.0', port=5000,  debug=False, threaded=True)


if __name__ == '__main__':
    # wee bit of overkill in our main definition here
    pyrs.start()
    # Define a depth stack which can be declared global
    depth_stack = Stack()

    thread.start_new_thread(threaded_stream, ())
    cnt = 0

    motorrunning = True

    # to do: take depth frame(s)
    # need to take time-relevant frames.
    # Pop periodically? can we pop-push the stack so we don't have a data backlog?
    # Analyse depth frames, generate vector
    # generate motor drive code

    while motorrunning:
        cnt += 1

        if cnt == 2:
            print("Driving forward")
            forwardDrive(200)

    #    if cnt == 150:
    #        print("slowing down")
            # forwardDrive(100)

    #    if cnt == 300:
    #        print("Driving backward")
            # backwardDrive(200)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            motorrunning=False
            break


    # time.sleep(1)
    print ("ended, stopping all motors")

    for i in range(1,5):
        imot = mh.getMotor(i)
        imot.run(Adafruit_MotorHAT.RELEASE)


    #shutdown_server(self)
