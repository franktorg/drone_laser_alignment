#!/usr/bin/env python

# Import libraries

import rospy # ROS interface
import tf
import numpy as np
import roslaunch
import math
from tf.transformations import quaternion_from_euler
from math import *
from sensor_msgs.msg import NavSatFix, Image, LaserScan
from geometry_msgs.msg import Point, PoseStamped, Quaternion, PoseWithCovarianceStamped
from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError # OpenCV with ROS
import cv2 # OpenCV
from fisheye_calib import get_ocam_model, cam2world, undistort_point, new_pixel_position


# Create general class with functions

class alignment:

    # Constructor
    def __init__(self):
        # Image Variables
        # Please make sure to intialze your image object with the right height and width
        self.image_height = 480 #1080
        self.image_width = 640 #1920
        self.image_step = 0
        self.cv_image = np.zeros((self.image_height, self.image_width, 3), np.uint8)
        self.black_and_white_image = np.zeros((self.image_height, self.image_width)\
                                              , np.uint8)
        self.coordinates = [[0,0]]
        # Camera calibration data
        ocam_mod = {}
        ocam_mod['c'] = float(0.7488)
        ocam_mod['d'] = float(0.962)
        ocam_mod['e'] = float(-0.1257)
        # Experimental center
        ocam_mod['cx'] = float(233) 
        ocam_mod['cy'] = float(305)
        self.ocam_model = ocam_mod
        #Intilize cvBridge Object
        self.bridge= CvBridge()

    #===================================================================================
    # Function to load image from ROS msg
    # Input: message from camera topic
    # Output: None
    #===================================================================================

    def load_image(self, msg):
        if msg is not None:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_height = msg.height
            self.image_width = msg.width
            self.image_step = msg.step
            self.black_and_white_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
          
            
    #===================================================================================
    # Function to filter images (Images preprocessing)
    # Input: Gray Image
    # Output: Filtered Image
    #===================================================================================

    def filter_image(self):

        kernel = np.ones((5,5), 'uint8') 

        (thresh, img_bin) = cv2.threshold(self.black_and_white_image, 24, 256\
                                                             , cv2.THRESH_BINARY_INV)
        
        # MedianBlur Filter (kernel_size = 13)
        img_mblur = cv2.medianBlur(img_bin, 13)
        
        # Erode filter
        img_erode = cv2.erode(img_mblur, kernel, iterations = 3)
        
        # Morphologic filter (Gradient)
        img_gradient = cv2.morphologyEx(img_erode, cv2.MORPH_GRADIENT, kernel)
             
        return img_gradient

    #===================================================================================
    # Function to detect blobs with custom parameters
    # Input: Images 
    # Output: Filtered image and keypoints
    #===================================================================================

    def param_blob_detector(self, filt_img):
        
        keypoints_num = 0

        # Define blob parameters 
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 10;
        params.maxThreshold = 200;

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 800

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.7

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.8
              
        # Create a detector with the given parameters acording to the openCv version
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv2.SimpleBlobDetector(params)
        else :
            detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs in the image
        keypoints = detector.detect(filt_img)

        # Draw detected keypoints as purple circles
        img_keypoints = cv2.drawKeypoints(filt_img, keypoints, np.array([]), (255,20,147)\
                                         , cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        #cv2.imshow("Keypoints", img_keypoints)
        #cv2.waitKey(3)

        if keypoints:
            keypoints_num = len(keypoints)
        
        return keypoints_num, keypoints, img_keypoints 

    #===================================================================================
    # Function to get coordinates from blob keypoints
    # Input: Keypoint(s)
    # Output: None 
    #         
    # Note: Change alignment attribute 'coordinates'(pixel coordinate) from each
    #       detected keypoint [x,y] 
    #===================================================================================

    def get_blob_coordinates(self, keypoints_num, keypoints):
        
        # No blobs detected in the image
        if keypoints_num < 1:
            self.coordinates = [[0,0]]
            print("No blobs detected")
            
        # One blob detected in the image
        elif keypoints_num == 1:
            x = keypoints[0].pt[0]
            y = keypoints[0].pt[1]
            self.coordinates = [[x, y]]
            print("Blob detected at:[", x, y,"]")
            
        # More than one blob detected in the image
        elif keypoints_num > 1:
            print(keypoints_num, "blobs detected")
            for i in range(keypoints_num):
                x = keypoints[i].pt[0] # keypoints [Keypoint No.].pt[x]
                y = keypoints[i].pt[1] # keypoints [Keypoint No.].pt[y]
                if i == 0:
                    self.coordinates[0][0] = x
                    self.coordinates[0][1] = y
                else:
                    self.coordinates.append([x, y])
                print("Blob number", i + 1 ,"detected at: [", x,y,"]")

       # print("")

# End alignment class


#===================================================================================
# Main function
#===================================================================================

def main():
    # Initiate ros node
    rospy.init_node('uav_alignment', anonymous=True)
    rospy.logwarn("Laser alignment node started")

    # Create alignment object
    uav = alignment()
    
    # Subscribe to camera Image topic  
    rospy.Subscriber("/usb_cam/image_raw",Image , uav.load_image)

    # Publisher of filtered image   
    img_publisher = rospy.Publisher("/blob_detector", Image, queue_size=10)

    # Keep ros node running
    while not rospy.is_shutdown():

        # Filter usb_cam image
        filtered_img = uav.filter_image()
        # Detect blobs
        keypoints_num, keypoints, pub_img = uav.param_blob_detector(filtered_img)
        # Get blobs coordinates
        uav.get_blob_coordinates(keypoints_num, keypoints)

        # Publish blobs in image    
        img_publisher.publish(uav.bridge.cv2_to_imgmsg(pub_img,"bgr8"))

        # New pixel coordinate
        for i in range(len(uav.coordinates)):
            vector_2D = new_pixel_position(uav.coordinates[i], uav.ocam_model)
            print('Vector: x =', vector_2D[0], ' y =', vector_2D[1])
            print("")
            

        #for x in range(O.imagewidth-1):
        # for y in range(O.imageheight-1):
        # if O.blackAndWhiteImage[y][x] != 0:
        # counter=counter+1
        #cv2.imshow("Image window", O.blackAndWhiteImage)
        #cv2.waitKey(3)
        #print(counter)
        #counter=0

        uav.coordinates = [[0,0]]
        #servo_pub = rospy.Publisher("/servo", UInt16, queue_size=10)


#===================================================================================
#***********************************************************************************
#************************************* Main ****************************************
#***********************************************************************************
#===================================================================================

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
