#!/usr/bin/env python
# script to push a series of stereo images in a loop
# sample operatin:
#   roscore
#   python pub_stereo.py
#   rosrun image_view image_view image:=/stereo/left/image_rect_color

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image
from stereo_msgs.msg import DisparityImage
import time
import yaml
from cv_bridge import CvBridge, CvBridgeError

def input_yaml(calib_file):
    with file(calib_file, 'r') as f:
        params = yaml.load(f)

    cam_info_l = CameraInfo()
    cam_info_l.height = params['size']['height']
    cam_info_l.width = params['size']['width']
    cam_info_l.distortion_model = 'plumb_bob'
    cam_info_l.K = params['M1']
    cam_info_l.D = params['D1']
    cam_info_l.P = params['P1']

    cam_info_r = CameraInfo()
    cam_info_r.height = params['size']['height']
    cam_info_r.width = params['size']['width']
    cam_info_r.distortion_model = 'plumb_bob'
    cam_info_r.K = params['M2']
    cam_info_r.D = params['D2']
    cam_info_r.P = params['P2']

    R = params['R']
    T = params['T']

    return cam_info_l, cam_info_r, R, T

# A yaml constructor is for loading from a yaml node.
# This is taken from: http://stackoverflow.com/a/15942429
def opencv_matrix_constructor(loader, node):
    mapping = loader.construct_mapping(node, deep=True)
    mat = np.array(mapping["data"])
    mat.resize(mapping["rows"], mapping["cols"])
    return mat
yaml.add_constructor(u"tag:yaml.org,2002:opencv-matrix", opencv_matrix_constructor)
 

if __name__ == '__main__':

    rate = 100

    yaml_file = "example.yaml"

    left_ci, right_ci, R, T = input_yaml(yaml_file)

    left_ci.R, right_ci.R, left_ci.P, right_ci.P, Q, roi1, roi2 = cv2.stereoRectify(
        left_ci.K, left_ci.D, right_ci.K, right_ci.D, (left_ci.width, left_ci.height), 
        R, T)

    mapx1, mapy1 = cv2.initUndistortRectifyMap(left_ci.K, left_ci.D, left_ci.R, 
        left_ci.P, (left_ci.width, left_ci.height), cv2.CV_32FC1)
    mapx2, mapy2 = cv2.initUndistortRectifyMap(right_ci.K, right_ci.D, right_ci.R, 
        right_ci.P, (left_ci.width, left_ci.height), cv2.CV_32FC1)


    pub_left_raw = rospy.Publisher('stereo' + '/' + 'left/image_raw', 
        Image, queue_size=10)
    pub_right_raw = rospy.Publisher('stereo' + '/' + 'right/image_raw', 
        Image, queue_size=10)
    pub_left_rect = rospy.Publisher('stereo' + '/' + 'left/image_rect_color', 
        Image, queue_size=10)
    pub_right_rect = rospy.Publisher('stereo' + '/' + 'right/image_rect_color', 
        Image, queue_size=10)
    pub_left_info = rospy.Publisher('stereo' + '/' + 'left/camera_info', 
        CameraInfo, queue_size=10)
    pub_right_info = rospy.Publisher('stereo' + '/' + 'right/camera_info', 
        CameraInfo, queue_size=10)
    # pub_disparity = rospy.Publisher('stereo' + '/' + 'disparity', 
        # DisparityImage, queue_size=10)
    pub_disparity = rospy.Publisher('stereo' + '/' + 'disparity', 
        Image, queue_size=10)

    rospy.init_node('stereo_cam')


    cap_l = cv2.VideoCapture(1)
    cap_r = cv2.VideoCapture(2)

    cap_l.set(cv2.CAP_PROP_FRAME_WIDTH, left_ci.width)
    cap_l.set(cv2.CAP_PROP_FRAME_HEIGHT, left_ci.height)
    cap_l.set(cv2.CAP_PROP_FPS, rate)

    cap_r.set(cv2.CAP_PROP_FRAME_WIDTH, right_ci.width)
    cap_r.set(cv2.CAP_PROP_FRAME_HEIGHT, right_ci.height)
    cap_r.set(cv2.CAP_PROP_FPS, rate)

    bridge = CvBridge()
    stereo = cv2.StereoBM_create(numDisparities=32, blockSize=21)

    #publish the synchronized messages until we are interrupted
    try:
        print 'publishing stereo images'
        while not rospy.is_shutdown():

            ret, img_l_raw = cap_l.read()
            ret, img_r_raw = cap_r.read()

            img_l_rect = cv2.remap(img_l_raw, mapx1, mapy1, cv2.INTER_LINEAR)
            img_r_rect = cv2.remap(img_r_raw, mapx2, mapy2, cv2.INTER_LINEAR)
            img_l_rect_bw = cv2.cvtColor(img_l_rect, cv2.COLOR_BGR2GRAY)
            img_r_rect_bw = cv2.cvtColor(img_r_rect, cv2.COLOR_BGR2GRAY)
            disparity = stereo.compute(img_l_rect_bw, img_r_rect_bw)

            disparity = cv2.convertScaleAbs(disparity)

            # cv2.imshow("left_raw", img_l_raw)
            # cv2.imshow("right_raw", img_r_raw)
            # cv2.imshow("left_rect", img_l_rect)
            # cv2.imshow("right_rect", img_r_rect)
            # cv2.imshow('disparity', disparity)
            # cv2.waitKey(1)

            stamp = rospy.Time.from_sec(time.time())

            img_l_raw_msg = bridge.cv2_to_imgmsg(img_l_raw, encoding="bgr8")
            img_r_raw_msg = bridge.cv2_to_imgmsg(img_r_raw, encoding="bgr8")
            img_l_raw_msg.header.stamp = stamp
            img_r_raw_msg.header.stamp = stamp

            #publish images
            pub_left_raw.publish(img_l_raw_msg)
            pub_right_raw.publish(img_r_raw_msg)
            pub_disparity.publish(bridge.cv2_to_imgmsg(disparity, encoding="mono8"))

            pub_left_rect.publish(bridge.cv2_to_imgmsg(img_l_rect, encoding="bgr8"))
            pub_right_rect.publish(bridge.cv2_to_imgmsg(img_r_rect, encoding="bgr8"))

            left_ci.header.stamp = stamp
            right_ci.header.stamp = stamp
            pub_left_info.publish(left_ci)
            pub_right_info.publish(right_ci)

            rospy.sleep(1.0/rate)

    except rospy.ROSInterruptException:
        pass