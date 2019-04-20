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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray

from dwa import *


rospy.init_node('path_planning', anonymous=True)
lr = tf.TransformListener()
br = tf.TransformBroadcaster()


class PathPlanningNode:
    def __init__(self):
            # print out a message for debugging
        rospy.loginfo("Starting path_planning_node (Python)")

        #self.safety_lock = False
        self.laser_data=None
        #self.manual_control=False
        #self.manual_control_msg=None

        self.count=0

        # listen to input drive commands and laser scans
        
        #apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, apriltag_callback, queue_size = 1)
        rospy.Subscriber("scan", LaserScan, self.laser_scan_callback)
        self.velcmd_pub = rospy.Publisher("/cmdvel", WheelCmdVel, queue_size = 1)

        # publish safe drive commands
        #self.safe_drive_pub = rospy.Publisher("racecar_planning", RacecarDriveStamped, queue_size=10)
        #self.safe_debug_pub = rospy.Publisher("racecar_planning_debug", Float64MultiArray, queue_size=10)

        # create a Timer to check for stalled inputs
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def laser_scan_callback(self, msg):
        # check laser scan points for safe condition?
        self.laser_data=msg
        # if min(msg.ranges) < 0.2:
        #     self.safety_lock = True
        # else:
        #     self.safety_lock = False
        # pass

    def timer_callback(self, event):
        # perhaps check for stalled inputs?
        #if self.manual_control:
        #    self.safe_drive_pub.publish(self.manual_control_msg)
        #    self.manual_control=False
        #    self.manual_control_msg=None
        #    return
        #else:
        self.move()
        return
        pass
    def move(self):
        #msg=RacecarDriveStamped(drive=RacecarDrive())
        #msg_debug=Float64MultiArray()
        wcv = WheelCmdVel()
        self.vel_desired=0.1
        debug=None
        if self.laser_data:
            alpha,debug=find_direction(self.laser_data,margin=0.23,stop_margin=0.23)
            if alpha is None:
                desiredWV_R,desiredWV_L=(0,0)
            else:
                desiredWV_R,desiredWV_L=alpha_to_w(alpha,self.vel_desired)
            wcv.desiredWV_R = desiredWV_R
            wcv.desiredWV_L = desiredWV_L
            self.velcmd_pub.publish(wcv)
            #servo=0.78
            #servo=servo
            #speed=0.0
            #msg_debug.data=debug
        else:
            # servo=0
            # speed=0
            wcv.desiredWV_R = 0.0
            wcv.desiredWV_L = 0.0
            self.velcmd_pub.publish(wcv) 
        if self.laser_data:
            print alpha
            print desiredWV_R,desiredWV_L
            print debug  
        return
	#self.safe_debug_pub.publish(msg_debug)
        #self.safe_drive_pub.publish(msg)
    
# def main():
#     #apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, apriltag_callback, queue_size = 1)
#     #self.laser_data=None
#     rospy.Subscriber("scan", LaserScan, self.laser_scan_callback)
    
#     rospy.sleep(1)
    
#     constant_vel = False
#     if constant_vel:
#         thread = threading.Thread(target = constant_vel_loop)
#     else:
#         thread = threading.Thread(target = navi_loop)
#     thread.start()
    
#     rospy.spin()

## sending constant velocity (Need to modify for Task 1)
def constant_vel_loop():
    velcmd_pub = rospy.Publisher('/cmdvel', WheelCmdVel, queue_size = 1)
    rate = rospy.Rate(100) # 100hz
    
    while not rospy.is_shutdown() :
        wcv = WheelCmdVel()
        wcv.desiredWV_R = 0.1
        wcv.desiredWV_L = 0.2
        
        velcmd_pub.publish(wcv) 
        
        rate.sleep() 

## apriltag msg handling function (Need to modify for Task 2)
def apriltag_callback(data):
    # use apriltag pose detection to find where is the robot
    for detection in data.detections:
        if detection.id == 1:   # tag id is the correct one
            poselist_tag_cam = pose2poselist(detection.pose)
            poselist_tag_base = transformPose(lr, poselist_tag_cam, 'camera', 'robot_base')
            poselist_base_tag = invPoselist(poselist_tag_base)
            poselist_base_map = transformPose(lr, poselist_base_tag, 'apriltag', 'map')
            pubFrame(br, pose = poselist_base_map, frame_id = '/robot_base', parent_frame_id = '/map')


## navigation control loop (No need to modify)
def navi_loop():
    velcmd_pub = rospy.Publisher("/cmdvel", WheelCmdVel, queue_size = 1)
    target_pose2d = [0.25, 0, np.pi]
    rate = rospy.Rate(100) # 100hz
    
    wcv = WheelCmdVel()
    
    arrived = False
    arrived_position = False
    
    while not rospy.is_shutdown() :
        # 1. get robot pose
        robot_pose3d = lookupTransform(lr, '/map', '/robot_base')
        
        if robot_pose3d is None:
            print '1. Tag not in view, Stop'
            wcv.desiredWV_R = 0.1  # right, left
            wcv.desiredWV_L = 0.1
            velcmd_pub.publish(wcv)  
            rate.sleep()
            continue
        
        robot_position2d  = robot_pose3d[0:2]
        target_position2d = target_pose2d[0:2]
        
        robot_yaw    = tfm.euler_from_quaternion(robot_pose3d[3:7]) [2]
        robot_pose2d = robot_position2d + [robot_yaw]
        
        # 2. navigation policy
        # 2.1 if       in the target, stop
        # 2.2 else if  close to target position, turn to the target orientation
        # 2.3 else if  in the correct heading, go straight to the target position,
        # 2.4 else     turn in the direction of the target position
        
        pos_delta         = np.array(target_position2d) - np.array(robot_position2d)
        robot_heading_vec = np.array([np.cos(robot_yaw), np.sin(robot_yaw)])
        heading_err_cross = cross2d( robot_heading_vec, pos_delta / np.linalg.norm(pos_delta) )
        
        # print 'robot_position2d', robot_position2d, 'target_position2d', target_position2d
        # print 'pos_delta', pos_delta
        # print 'robot_yaw', robot_yaw
        # print 'norm delta', np.linalg.norm( pos_delta ), 'diffrad', diffrad(robot_yaw, target_pose2d[2])
        # print 'heading_err_cross', heading_err_cross
        



        
        # if arrived or (np.linalg.norm( pos_delta ) < 0.08 and np.fabs(diffrad(robot_yaw, target_pose2d[2]))<0.05) :
        #     print 'Case 2.1  Stop'
        #     wcv.desiredWV_R = 0  
        #     wcv.desiredWV_L = 0
        #     arrived = True
        # elif np.linalg.norm( pos_delta ) < 0.08:
        #     arrived_position = True
        #     if diffrad(robot_yaw, target_pose2d[2]) > 0:
        #         print 'Case 2.2.1  Turn right slowly'      
        #         wcv.desiredWV_R = -0.05 
        #         wcv.desiredWV_L = 0.05
        #     else:
        #         print 'Case 2.2.2  Turn left slowly'
        #         wcv.desiredWV_R = 0.05  
        #         wcv.desiredWV_L = -0.05
        # elif arrived_position or np.fabs( heading_err_cross ) < 0.2:
        #     print 'Case 2.3  Straight forward'  
        #     wcv.desiredWV_R = 0.1
        #     wcv.desiredWV_L = 0.1
        # else:
        #     if heading_err_cross < 0:
        #         print 'Case 2.4.1  Turn right'
        #         wcv.desiredWV_R = -0.1
        #         wcv.desiredWV_L = 0.1
        #     else:
        #         print 'Case 2.4.2  Turn left'
        #         wcv.desiredWV_R = 0.1
        #         wcv.desiredWV_L = -0.1

        # 2-axis Proportional Control Implementation
        dX = np.linalg.norm(pos_delta)
        dTheta = heading_err_cross
        vel_desired =  -(0.1 - dX)*0.25
        angVel_desired = (0 - dTheta)*0.25
        wcv.desiredWV_R = vel_desired - angVel_desired
        wcv.desiredWV_L = vel_desired + angVel_desired


                
        velcmd_pub.publish(wcv)  
        
        rate.sleep()

if __name__=='__main__':
    # main()
    #rospy.init_node("path_planning_node")

    node = PathPlanningNode()

    rospy.spin()
    
