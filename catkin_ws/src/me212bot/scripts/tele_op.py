#!/usr/bin/python

# 2.12 Lab 3 AprilTag Navigation: use AprilTag to get current robot (X,Y,Theta) in world frame, and to navigate to target (X,Y,Theta)
# Peter Yu Sept 2016

import rospy
import tf
import numpy as np
import threading
import serial
import tf.transformations as tfm

from me212bot.msg import WheelCmdVel
from apriltags.msg import AprilTagDetections
from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad


rospy.init_node('tele_op', anonymous=True)
lr = tf.TransformListener()
br = tf.TransformBroadcaster()

class Cmd(object):
    """docstring for Cmd"""
    def __init__(self):
        super(Cmd, self).__init__()
        # self.arg = arg
        self.turn_direction=1
        self.turn_cd=0
        self.run_dir=0

self=Cmd()

    
def main():
    #apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, apriltag_callback, queue_size = 1)
    
    rospy.sleep(1)
    
    constant_vel = True
    cmd_vel_loop()
    # if constant_vel:
    #     thread = threading.Thread(target = constant_vel_loop)
    # else:
    #     thread = threading.Thread(target = navi_loop)
    # thread.start()
    self.turn_direction=1
    self.turn_cd=0
    self.run_dir=0
    
    rospy.spin()

## sending constant velocity (Need to modify for Task 1)
def cmd_vel_loop():
    velcmd_pub = rospy.Publisher('/cmdvel', WheelCmdVel, queue_size = 1)
    rate = rospy.Rate(100) # 100hz
    
    while not rospy.is_shutdown() :
        wcv = WheelCmdVel()
        cmd=raw_input()
        
        #cmd='w'
        #print 'test'
        if cmd=='w':
            self.run_dir=1.
            # wcv.desiredWV_R = 0.1
            # wcv.desiredWV_L = 0.1
            #print 'haha'
        elif cmd=='a':
            self.turn_direction=-1
            self.turn_cd=4
            
        elif cmd=='d':
            self.turn_direction=-1
            self.turn_cd=4
            
        elif cmd=='s':
            self.run_dir=-1.
            
        else:
            self.run_dir=0.

        if self.turn_cd>0:
            if self.turn_direction==-1:
                wcv.desiredWV_R = 0.2*self.run_dir
                wcv.desiredWV_L = 0.1*self.run_dir
            elif self.turn_direction==1:
                wcv.desiredWV_R = 0.1*self.run_dir
                wcv.desiredWV_L = 0.2*self.run_dir
            else:
                wcv.desiredWV_R = 0.
                wcv.desiredWV_L = 0.
            self.turn_cd-=1
        elif self.run_dir>0:
            wcv.desiredWV_R = 0.1
            wcv.desiredWV_L = 0.1
        elif self.run_dir<0:
            wcv.desiredWV_R = 0.
            wcv.desiredWV_L = 0.
        else:
            wcv.desiredWV_R = -0.1
            wcv.desiredWV_L = -0.1

        print wcv
        velcmd_pub.publish(wcv) 
        
        rate.sleep() 

## apriltag msg handling function (Need to modify for Task 2)
# def apriltag_callback(data):
#     # use apriltag pose detection to find where is the robot
#     for detection in data.detections:
#         if detection.id == 1:   # tag id is the correct one
#             poselist_tag_cam = pose2poselist(detection.pose)
#             poselist_tag_base = transformPose(lr, poselist_tag_cam, 'camera', 'robot_base')
#             poselist_base_tag = invPoselist(poselist_tag_base)
#             poselist_base_map = transformPose(lr, poselist_base_tag, 'apriltag', 'map')
#             pubFrame(br, pose = poselist_base_map, frame_id = '/robot_base', parent_frame_id = '/map')



if __name__=='__main__':
    main()
    
