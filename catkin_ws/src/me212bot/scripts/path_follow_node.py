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
from geometry_msgs.msg import Pose, Quaternion, PoseStamped

from dwa import *
from traj_follow import *

rospy.init_node('path_follow',anonymous=True)

class State(object):
    """docstring for State"""
    def __init__(self):
        super(State, self).__init__()
        #self.arg = arg

        self.laser_data=None
        self.x=None
        self.y=None
        self.yaw=None

        self.encoder_distance=0

        rospy.Subscriber("scan", LaserScan, self.laser_scan_callback)

        rospy.Subscriber("/pose", RobotPose, self.pose_calback)
        rospy.Subscriber("/odom_encoder", WheelEncoder,self.encoder_callback, queue_size = 1)
        self.tag_num=8
        self.seen=[None]*self.tag_num



        tag_id_list=range(self.tag_num)
        for tag_id in tag_id_list:
            rospy.Subscriber("/apriltag_pose_%d"%tag_id, PoseStamped,self.april_tag_callback,tag_id, queue_size = 1)
        pass

    def laser_scan_callback(self, msg):
        # check laser scan points for safe condition?
        self.laser_data=msg

    def pose_calback(self,msg):
        self.x=msg.x
        self.y=msg.y
        self.yaw=msg.yaw
        self.err=msg.err
        self.timestamp=msg.Time_Stamp

    def april_tag_callback(self,data,tag_id):
        self.seen[tag_id]=True
        # state.tag_time=data.header.stamp.to_sec()
        # data=pose2poselist(data.pose)
        # x,y=data[0:2]
        # theta=tfm.euler_from_quaternion(data[3:7])[2]
        # state.tag_data[tag_id]=[x,y,theta]
        #state.tag_measure_time=data.header.stamp.to_sec()

    def clear_seen(self):
        self.seen=[None]*self.tag_num

    def encoder_callback(self,data):
        self.encoder_distance=data.distance
        #state.encoder_data.append(data)
        pass

        pass

velcmd_pub = rospy.Publisher("/cmdvel", WheelCmdVel, queue_size = 1)

def pub_vel(v,dtheta):
    ss=v*2
    mm=dtheta
    tr=(ss+mm)/2
    tl=(ss-mm)/2
    wcv = WheelCmdVel()
    wcv.desiredWV_R = tr
    wcv.desiredWV_L = tl
    velcmd_pub.publish(wcv)
    #print wcv


def rotate(target_yaw,threshold=np.deg2rad(1.)):
    print 'rotate to: '+str(target_yaw)
    pid=(0.5,0.1,0.)
    previous_err=[]
    while True:
        err=pi_2_pi(state.yaw-target_yaw)

        if abs(err)<threshold:
            pub_vel(0,0)
            print 'arrived at target_yaw'
            return err
        
        #if abs(err)<np.deg2rad(10.):
        #    pid=(0.2,0.01,0.1)
        ss,_=PID(err,0.,previous_err,pid=pid)
        ss=max(0.2,abs(ss))
        #else:
        #    ss=0.5
        print 'err='+str(err)
        if err>0:
            pub_vel(0,-ss)
        else:
            pub_vel(0,ss)
        rospy.sleep(0.1)

def cal_angle(pose,target):
    dx=target[0]-pose[0]
    dy=target[1]-pose[1]
    angle=pi_2_pi(-np.arctan2(dx,dy))
    return angle


def move_to_target(target,v=0.5,stopmargin=None):
    #target0=target
    yaw_move=cal_angle([state.x,state.y],target)
    
    rotate(yaw_move)
    target_temp=[target[0],target[1],state.yaw]
    pid=(0.2,0.01,0.1)
    previous_err=[]
    #return
    #for i in range(10):
    while True:
        x=state.x
        y=state.y
        yaw=state.yaw
        print x,y,target_temp
        dis_err=distance_to_target(x,y,target_temp)
        print 'dis_err=',dis_err
        if dis_err<=0.005:
            print 'arrived at',target[0:2]
            pub_vel(0.,0.)
            break

        if (not (stopmargin is None)) and state.laser_data:
            while True:
                alpha,distance,debug=find_direction_rot(state.laser_data,angle_list=[0.],margin=stopmargin)
                if distance[0]>0:
                    break
                print 'obstacle detected stoped'

        err=get_error(x,y,target_temp)
        angle_err=get_error_angle(x,y,yaw,target_temp)

        dtheta,pp=PID(err,angle_err,previous_err,pid=pid)
        print err,angle_err,pp,dtheta

        pub_vel(v,dtheta)
        rospy.sleep(0.1)
    rotate(target[2])
    return

def dwa_distance(dis,vel_desired=0.1,stop_margin=0.05):
    encoder0=state.encoder_distance
    while True:
        if self.laser_data:
            break
    while True:
        if state.encoder_distance-encoder0>=dis:
            pub_vel(0,0)
            break
        
        
        
        alpha,distance,debug=find_direction(state.laser_data,margin=0.3)
        #alpha_rot,distance_rot,debug_rot=find_direction_rot(self.laser_data,margin=0.4)
        if distance<=stop_margin:
            pub_vel(0,0)
            break
        wcv = WheelCmdVel()
        desiredWV_R,desiredWV_L=alpha_to_w(alpha,self.vel_desired)
        wcv.desiredWV_R = desiredWV_R
        wcv.desiredWV_L = desiredWV_L
        self.velcmd_pub.publish(wcv)
        rospy.sleep(0.1)

def move_forward(dis,vel_desired=0.1,stop_margin=0.05):
    encoder0=state.encoder_distance
    while True:
        if state.encoder_distance-encoder0>=dis:
            pub_vel(0,0)
            break
        alpha,distance,debug=find_direction(state.laser_data,angle_list=[0.],margin=0.3)
        if distance<=stop_margin:
            pub_vel(0,0)
            break
        pub_vel(vel_desired,0)
        rospy.sleep(0.1)

def move_backwward(dis,vel_desired=0.1):
    encoder0=state.encoder_distance
    while True:
        if state.encoder_distance-encoder0>=dis:
            pub_vel(0,0)
            break
        #alpha,distance,debug=find_direction(state.laser_data,angle_list=[0.],margin=0.3)
        # if distance<=stop_margin:
        #     pub_vel(0,0)
        #     break
        pub_vel(-vel_desired,0)
        rospy.sleep(0.1)

def move_to_target_reverse(target,v=0.5):
    #target0=target
    #target[2]=-target[2]
    yaw_move=pi_2_pi(cal_angle([state.x,state.y],target)+np.pi)
    
    rotate(yaw_move)
    yaw=pi_2_pi(state.yaw+np.pi)
    target_temp=[target[0],target[1],yaw]
    pid=(0.2,0.01,0.1)
    previous_err=[]
    #return
    #for i in range(10):
    while True:
        x=state.x
        y=state.y
        yaw=pi_2_pi(state.yaw+np.pi)
        print x,y,target_temp
        dis_err=distance_to_target(x,y,target_temp)
        print 'dis_err=',dis_err
        if dis_err<=0.005:
            print 'arrived at',target[0:2]
            pub_vel(0.,0.)
            break

        err=get_error(x,y,target_temp)
        angle_err=get_error_angle(x,y,yaw,target_temp)

        dtheta,pp=PID(err,angle_err,previous_err,pid=pid)
        print err,angle_err,pp,dtheta

        pub_vel(-v,dtheta)
        rospy.sleep(0.1)
    
    rotate(target[2])
    return



state=State()

seq_list=[[1.952,1.75,0.],
        [1.952,1.15,0.],
        ]

def Move():
    # while True:
    #     if not (state.yaw is None):break
    # move_to_target(seq_list[0])
    # rospy.sleep(1.)
    # move_to_target_reverse(seq_list[1])

    while True:
        if state.laser_data:
            break
    direction,distance,_=find_direction_rot(state.laser_data,np.linspace(-80/180.*np.pi,80/180.*np.pi,15),margin=0.2)
    print direction,distance
    pass

def main():
    thread = threading.Thread(target = Move)

    thread.start()

    rospy.spin()
    pass

if __name__ == '__main__':
    main()


