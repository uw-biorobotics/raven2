# Stereo Cameras

This folder contains a python script for publishing images from two usb cameras with known calibration matrices. The script parses a .yaml file, rectifies raw images, publishes, and depends on OpenCV and Numpy.

## Topics published:

* /stereo/(left & right)/image_raw

* /stereo/(left & right)/image_rect_color

* /stereo/(left & right)/camera_info

* /stereo/disparity

## pub_stereo.py should be used as follows:

* Update the ``rate`` variable with the frame rate of your cameras.

* Update the ``yaml_file`` variable with the name of your .yaml file containing camera matrices in the same format as example.yaml.

* Ensure the left stereo camera is dev1, and right camera dev2.

## Sample operation:

* In separate terminals, execute the following commands:

  * roscore

  * python pub_stereo.py

  * rosrun image_view image_view image:=/stereo/left/image_rect_color
