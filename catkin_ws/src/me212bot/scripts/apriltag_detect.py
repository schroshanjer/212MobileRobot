#!/usr/bin/python

import rospy
import tf
import numpy as np
import threading
import serial
import tf.transformations as tfm

from me212bot.msg import WheelCmdVel
from apriltags.msg import AprilTagDetections
from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad, poselist2pose
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from std_msgs.msg import Header


rospy.init_node('apriltag_detect', anonymous=True)
lr = tf.TransformListener()
br = tf.TransformBroadcaster()

tag_id_list=range(5)
tag_pubs=[]
for tag_id in tag_id_list:
    tag_pubs.append(rospy.Publisher("/apriltag_pose_%d"%tag_id, PoseStamped, queue_size = 1))

def main():
    apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, apriltag_callback, queue_size = 1)
    
    rospy.sleep(0.1)

    rospy.spin()

def apriltag_callback(data):
    tt=rospy.Time.now()
    # use apriltag pose detection to find where is the robot
    for detection in data.detections:
        #if detection.id == 1:   # tag id is the correct one
        poselist_tag_cam = pose2poselist(detection.pose)
        poselist_tag_base = transformPose(lr, poselist_tag_cam, 'camera', 'robot_base')
        #print poselist_tag_base
        #print type(detection.id)
        tag_pose=poselist2pose(poselist_tag_base)
        if detection.id in tag_id_list:
            pose_stamp=PoseStamped()
            pose_stamp.header.stamp = tt
            pose_stamp.header.frame_id = "/base"
            pose_stamp.pose=tag_pose
            tag_pubs[detection.id].publish(pose_stamp)
        #poselist_base_tag = invPoselist(poselist_tag_base)
        #poselist_base_map = transformPose(lr, poselist_base_tag, 'apriltag', 'map')
        #pubFrame(br, pose = poselist_base_map, frame_id = '/robot_base', parent_frame_id = '/map')

if __name__=='__main__':
    main()