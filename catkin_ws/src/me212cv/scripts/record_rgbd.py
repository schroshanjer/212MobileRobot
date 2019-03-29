#!/usr/bin/python
import rospy
import numpy as np
from numpy.linalg import inv
import cv2  # OpenCV module

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math
import cPickle as pkl
import time,os

outputpath='./'

rospy.init_node('object_detection', anonymous=True)

# Publisher for publishing pyramid marker in rviz
vis_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10) 

# Publisher for publishing images in rviz
img_pub1 = rospy.Publisher('/object_detection/image_with_cross', Image, queue_size=10)
img_pub2 = rospy.Publisher('/object_detection/mask_eroded', Image, queue_size=10)
img_pub3 = rospy.Publisher('/object_detection/mask_eroded_dilated', Image, queue_size=10)
img_pub4 = rospy.Publisher('/object_detection/img_result', Image, queue_size=10)

# Bridge to convert ROS Image type to OpenCV Image type
cv_bridge = CvBridge()  

# Get the camera calibration parameter for the rectified image
msg = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo, timeout=None) 
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]

fx = msg.P[0]
fy = msg.P[5]
cx = msg.P[2]
cy = msg.P[6]

def main():
    image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
    depth_sub = message_filters.Subscriber("/camera/depth_registered/image", Image)

    ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.5)
    ts.registerCallback(rosRGBDCallBack)

    rospy.spin()

def rosRGBDCallBack(rgb_data, depth_data):
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        cv_image2 = np.array(cv_image, dtype=np.float32)
        cv_depthimage = cv_bridge.imgmsg_to_cv2(depth_data, "32FC1")
        cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)
    except CvBridgeError as e:
        print(e)
    output(cv_image2,cv_depthimage2)

def output(rgb_data, depth_data):
    tt=time.time()
    filename=str(int(tt*10**6))+'.pkl'
    with open(os.path.join(outputpath,filename),'wb') as f:
        data={'timestamp':tt,'rgb':rgb_data,'depth':depth_data}
        pkl.dump(data,f)


    #contours, mask_image = HSVObjectDetection(cv_image, toPrint = False)

    # for cnt in contours:
    #     xp,yp,w,h = cv2.boundingRect(cnt)
        
    #     # Get depth value from depth image, need to make sure the value is in the normal range 0.1-10 meter
    #     if not math.isnan(cv_depthimage2[int(yp)][int(xp)]) and cv_depthimage2[int(yp)][int(xp)] > 0.1 and cv_depthimage2[int(yp)][int(xp)] < 10.0:
    #         zc = cv_depthimage2[int(yp)][int(xp)]
    #         print 'zc', zc
    #     else:
    #         continue
            
    #     centerx, centery = xp+w/2, yp+h/2
    #     cv2.rectangle(cv_image,(xp,yp),(xp+w,yp+h),[0,255,255],2)
        
    #     #showPyramid(centerx, centery, zc, w, h)

if __name__=='__main__':
    main()

