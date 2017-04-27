#!/usr/bin/env python
# -*- coding: utf-8 -*-

## setup logging
import logging
logging.basicConfig(level = logging.INFO)

## import the package
import pyrealsense as pyrs

## start the service
pyrs.start()

## create a device from device id and streams of interest
cam = pyrs.Device(device_id = 0, streams = [pyrs.DepthStream(fps = 60)])

## wait for data and retrieve numpy array for ~1 second
for i in range(60):
    cam.wait_for_frame()
    print(cam.depth)
