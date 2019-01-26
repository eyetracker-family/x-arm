#!/bin/sh

#roslaunch lighthouse eyetracker.launch &

#sleep 2 && roslaunch macaca_tf_setup tf_setup.launch &

roslaunch usbcam usbcam_eye.launch id0:=1 id1:=0 reso:=720& 

sleep 1 && roslaunch usbcam usbcam_scene.launch id0:=3 id1:=2 reso:=720&

sleep 1 && roslaunch eyetracking double_pupiltracking.launch & 

sleep 2 && rviz
