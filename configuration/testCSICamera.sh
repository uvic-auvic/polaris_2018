#!/bin/bash

gst-launch-1.0 nvcamerasrc ! 'video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)I420, framerate=(fraction)60/1' ! nvvidconv ! 'video/x-raw(memory:NVMM), format=(string)I420' ! nvoverlaysink -e
