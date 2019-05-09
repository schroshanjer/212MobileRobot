#!/usr/bin/python

import rospy
import tf
import numpy as np
import threading
import serial
import tf.transformations as tfm

from me212bot.msg import WheelCmdVel,WheelEncoder,RobotPose
from apriltags.msg import AprilTagDetections
from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray

from dwa import *

class State(object):
    """docstring for State"""
    def __init__(self):
        super(State, self).__init__()
        #self.arg = arg

        self.laser_data=None
        self.x=None
        self.y=None
        self.yaw=None

        rospy.Subscriber("scan", LaserScan, self.laser_scan_callback)

        rospy.Subscriber("/pose", RobotPose, self.pose_callback)

    def laser_scan_callback(self, msg):
        # check laser scan points for safe condition?
        self.laser_data=msg

    def pose_calback(self,msg):
        self.x=msg.x
        self.y=msg.y
        self.yaw=msg.yaw
        self.err=msg.err
        self.timestamp=msg.TimeStamp

velcmd_pub = rospy.Publisher("/cmdvel", WheelCmdVel, queue_size = 1)

def pub_vel(v,dtheta):
    ss=v*2
    mm=dtheta
    tr=(ss-mm)/2
    tl=(ss+mm)/2
    wcv = WheelCmdVel()
    wcv.desiredWV_R = tr
    wcv.desiredWV_L = tl
    velcmd_pub.publish(wcv)


def rotate(target_yaw,threshold=np.deg2rad(2.)):
    while True:
        err=state.yaw-target_yaw

        if abs(err)<threshold:
            return err

        if abs(err)<np.deg2rad(10.):
            ss=0.1
        else:
            ss=0.2

        if err>0:
            pub_vel(0,-ss)
        else:
            pub_vel(0,ss)

def cal_angle(pose,target):
    dx=target[0]-pose[0]
    dy=target[1]-pose[1]
    angle=pi_2_pi(-np.arctan2(dx,dy))
    return angle


def move_to_target(target,v=0.15,reverse=False):
    #target0=target
    yaw_move=cal_angle([state.x,state.y],target)
    target_temp=[target[0],target[1],yaw_move]
    rotate(yaw_move)
    pid=(0.4,0.01,0.1)
    previous_err=[]
    while True:
        x=state.x
        y=state.y
        yaw=state.yaw

        dis_err=distance_to_target(x,y,target)
        if dis_err<=0.02:
            break

        err=get_error(x,y,target_temp)
        angle_err=get_error_angle(x,y,yaw,target_temp)

        dtheta,_=PID(err,angle_err,previous_err,pid=pid)

        pub_vel(v,dtheta)
    rotate(target[2])
    return



state=State()

seq_list=[[1,1,np.pi]]

def Move():
    while True:
        if state.yaw:break
    move_to_target(seq_list[0])

    pass

def main():
    thread = threading.Thread(target = Move)

    thread.start()

    rospy.spin()
    pass

if __name__ == '__main__':
    main()


