#!/usr/bin/python

import rospy
import threading

import tf.transformations as tfm
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from std_msgs.msg import Header


import numpy as np

import traceback,time

from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad, poselist2pose
from me212bot.msg import WheelCmdVel,WheelEncoder,RobotPose

from extended_kalman_filter import *

rospy.init_node('localization',anonymous=True)

pub_pose=rospy.Publisher("/pose", RobotPose, queue_size = 1)

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

class State(object):
    """docstring for State"""
    def __init__(self):
        super(State, self).__init__()
        #self.arg = arg

        self.encoder_data=[]
        self.last_encoder=None
        self.last_encoder_time=None

        self.tag_num=2
        #self.tag_pose=np.array([0,0,-np.pi])
        self.tag_data=[None]*self.tag_num
        self.tag_time=None
        self.last_tag_time=None

        self.lm=np.array([[2,3,1.71],
                  [0.,0.,np.pi],
        ])

        

        #switch x,y
        tt=np.array([[0.,1.,.0],[1.,0.,0.],[0.,0.,1.]])

        self.lm=self.lm.dot(tt)



        self.pose=np.zeros(3)
        self.xEst = np.zeros((4, 1))
        self.PEst = np.diag([0.1,0.1,np.pi/18.,0.1])**2

    def clear_measure(self):
        if self.encoder_data:
            self.last_encoder=self.encoder_data[-1]
            self.last_encoder_time=self.encoder_data[-1].Time_Stamp
            if self.tag_time and self.tag_time>self.last_encoder_time:
                self.last_encoder_time=self.tag_time
        elif self.tag_time:
            self.last_encoder_time=self.tag_time


        self.last_tag_time=self.tag_time
        self.encoder_data=[]
        self.tag_data=[None]*self.tag_num
        self.tag_time=None

state=State()

def get_u(dt,dTheta_L,dTheta_R):
    b = 0.225
    r = 0.037
    if dt==0:
        return np.array([[0.,0.]]).T
    v= (dTheta_L+dTheta_R)/2*r/dt
    v2= (dTheta_R-dTheta_L)*r/b/dt
    return np.array([[v,v2]]).T



def april_tag_callback(data,tag_id):
    state.tag_time=data.header.stamp.to_sec()
    data=pose2poselist(data.pose)
    x,y=data[0:2]
    theta=tfm.euler_from_quaternion(data[3:7])[2]
    state.tag_data[tag_id]=[x,y,theta]
    #state.tag_measure_time=data.header.stamp.to_sec()

    pass

def encoder_callback(data):

    state.encoder_data.append(data)
    pass

def observation(lm,tag_data):
    z=[]
    for i in range(state.tag_num):
        if tag_data[i]:
            x=lm[i,0]-tag_data[i][0]
            y=lm[i,1]-tag_data[i][1]
            print i, lm[i,2],tag_data[i]
            pose=pi_2_pi(lm[i,2]-(tag_data[i][2]-np.pi/2))
            print pose
            z.append(np.array([x,y,pose,i]).reshape(-1,1))
    return z

def pubilish_pose(xEst,PEst,t):
    pose=RobotPose()
    pose.Time_Stamp=t
    #switch x,y
    pose.x=xEst[1]
    pose.y=xEst[0]
    pose.yaw=xEst[2]

    # err=np.zeros((3,3))
    # err[0,0]=PEst[1,1]
    # err[0,1]=PEst[0,1]
    # err[1,0]=PEst[1,0]
    # err[]
    tt=np.array([[0.,1.,.0],[1.,0.,0.],[0.,0.,1.]])

    #switch x,y
    pose.err=(tt.dot(PEst[0:3,0:3]).dot(tt)).reshape(-1)

    pub_pose.publish(pose)
    return


def Localization():
    # last_encoder_time=0
    # last_encoder=[]
    while True:
        rospy.sleep(0.1)
        #print state.encoder_data
        encoder_list=state.encoder_data
        tag_data=state.tag_data
        tag_time=state.tag_time
        last_tag_time=state.tag_data
        last_encoder=state.last_encoder
        last_encoder_time=state.last_encoder_time

        if not encoder_list and not tag_time:
            continue

        xEst=state.xEst
        PEst=state.PEst
        lm=state.lm

        # if not last_encoder_time:
        #     print 'no previous encoder, skip this step'
        if not last_encoder:
            if encoder_list:
                last_encoder=encoder_list[0]
                last_encoder_time=encoder_list[0].Time_Stamp
            else:
                last_encoder_time=tag_time
            #state.clear_measure()
        #last_encoder_time=state.last_encoder
        #print state.tag_data
        #print state.tag_time
        state.clear_measure()


        encoder_time_list=np.array([ec.Time_Stamp for ec in encoder_list])
        if tag_time: 
            encoder_list_1=[]
            encoder_list_2=[]
            
            for ii in range(len(encoder_time_list)):
                if encoder_time_list[ii]<=tag_time:
                    encoder_list_1.append(encoder_list[ii])
                else:
                    encoder_list_2.append(encoder_list[ii])
                
            #encoder_list_1=encoder_list[np.where(encoder_time_list<=tag_time)]

            #encoder_list_2=encoder_list[np.where(encoder_time_list>tag_time)]
        else:
            encoder_list_1=encoder_list
            encoder_list_2=[]

        if encoder_list_1:
            dt=encoder_list_1[0].Time_Stamp-last_encoder_time
            
            dTheta_L=encoder_list_1[0].Theta_L-last_encoder.Theta_L
            dTheta_R=encoder_list_1[0].Theta_R-last_encoder.Theta_R

            u=get_u(dt,dTheta_L,dTheta_R)
            
            #print u
            
            xEst, PEst=ekf_update(xEst, PEst, [], u,lm,dt)
            

            last_encoder_time=encoder_list_1[0].Time_Stamp
            last_encoder=encoder_list_1[0]

            for i in range(1,len(encoder_list_1)):
                dt=encoder_list_1[i].Time_Stamp-encoder_list_1[i-1].Time_Stamp
                dTheta_L=encoder_list_1[i].Theta_L-encoder_list_1[i-1].Theta_L
                dTheta_R=encoder_list_1[i].Theta_R-encoder_list_1[i-1].Theta_R
                u=get_u(dt,dTheta_L,dTheta_R)
                xEst, PEst=ekf_update(xEst, PEst, [], u,lm,dt)
                last_encoder_time=encoder_list_1[i].Time_Stamp
                last_encoder=encoder_list_1[i]

        if tag_time:
            dt=encoder_list_1[0].Time_Stamp-last_encoder_time
            if encoder_list_2:
                dt2=encoder_list_2[0].Time_Stamp-tag_time
                dTheta_L=encoder_list_2[0].Theta_L-last_encoder.Theta_L
                dTheta_R=encoder_list_2[0].Theta_R-last_encoder.Theta_R

                dTheta_L=dTheta_L/(dt+dt2)*dt
                dTheta_R=dTheta_R/(dt+dt2)*dt
            else:
                dTheta_L=0.
                dTheta_R=0.
            u=get_u(dt,dTheta_L,dTheta_R)

            z=observation(lm,tag_data)
            #print 'before',xEst
            xEst, PEst=ekf_update(xEst, PEst, z, u,lm,dt)
            #print 'after',xEst
            # last_encoder_time=encoder_list_1[i].Time_Stamp
            # last_encoder=encoder_list_1[i]

        if encoder_list_2:
            dt=encoder_list_2[0].Time_Stamp-last_encoder_time
            
            dTheta_L=encoder_list_2[0].Theta_L-last_encoder.Theta_L
            dTheta_R=encoder_list_2[0].Theta_R-last_encoder.Theta_R

            u=get_u(dt,dTheta_L,dTheta_R)

            xEst, PEst=ekf_update(xEst, PEst, [], u,lm,dt)

            last_encoder_time=encoder_list_2[0].Time_Stamp
            last_encoder=encoder_list_2[0]

            for i in range(1,len(encoder_list_2)):
                dt=encoder_list_2[i].Time_Stamp-encoder_list_2[i-1].Time_Stamp
                dTheta_L=encoder_list_2[i].Theta_L-encoder_list_2[i-1].Theta_L
                dTheta_R=encoder_list_2[i].Theta_R-encoder_list_2[i-1].Theta_R
                u=get_u(dt,dTheta_L,dTheta_R)
                xEst, PEst=ekf_update(xEst, PEst, [], u,lm,dt)
                last_encoder_time=encoder_list_2[i].Time_Stamp
                last_encoder=encoder_list_2[i]
        
        print xEst
        print PEst
        state.xEst=xEst
        state.PEst=PEst

        pubilish_pose(xEst,PEst,last_encoder_time)



        pass

def main():
    thread = threading.Thread(target = Localization)
    
    tag_id_list=range(state.tag_num)
    for tag_id in tag_id_list:
        rospy.Subscriber("/apriltag_pose_%d"%tag_id, PoseStamped,april_tag_callback,tag_id, queue_size = 1)
    pass

    rospy.Subscriber("/odom_encoder", WheelEncoder,encoder_callback, queue_size = 1)

    thread.start()

    rospy.spin()


if __name__ == '__main__':
    main()




        
