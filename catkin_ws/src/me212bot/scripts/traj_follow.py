import numpy as np
from math import sqrt
import math

import os


# last_target_index=[0]
# now_target_index=[1]

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def PID(err,angle_err,previous_err,pid=(0.68,0.02,0.44)):
    kp=pid[0]
    ki=pid[1]
    kd=pid[2]

    p=err
    if len(previous_err):
        i=np.array(previous_err).sum()/len(previous_err)
    else:
        i=0.0
    d=angle_err
    pid=(p,i,d)

    previous_err.append(err)

    return kp*p+ki*i+kd*d,pid

    pass

def getLinePara(target):
    x=target[0]
    y=target[1]
    theta=target[2]
    #line[1]=
    a = np.cos(theta)#line[0][1] - line[1][1]
    b = np.sin(theta)#line[1][0] - line[0][0]
    # c = line[0][0] *line[1][1] - line[1][0] * line[0][1]
    c= -a*x-b*y
    return a,b,c

def get_error(x,y,target):
    #print now_target_index[0]
    # tt=trajs[now_target_index[0]]
    # tl=trajs[last_target_index[0]]
    #print tt.rot_center
    a,b,c=getLinePara(target)

    #print a,b,c
    d=(a*x+b*y+c)/(a**2+b**2)

    return d

def distance_to_target(x,y,target):
    x0=target[0]
    y0=target[1]
    theta=target[2]

    a = -np.sin(theta)#line[0][1] - line[1][1]
    b = np.cos(theta)#line[1][0] - line[0][0]
    # c = line[0][0] *line[1][1] - line[1][0] * line[0][1]
    c= -a*x0-b*y0

    d=(a*x+b*y+c)/(a**2+b**2)

    return -d

    pass


def get_error_angle(x,y,yaw,target):
    theta=target[2]

    err=-pi_2_pi(yaw-theta)
    return err

    pass


if __name__ == '__main__':

    target=[2,2,np.pi/4-np.pi]

    print get_error(1,1,target)

    print np.rad2deg(get_error_angle(1,1,np.deg2rad(30),target))
