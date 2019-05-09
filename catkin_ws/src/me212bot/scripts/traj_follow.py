import numpy as np
from math import sqrt
import math

import os


# last_target_index=[0]
# now_target_index=[1]

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

# def cal_angle(p3,p1,p2):
#     t1=np.arctan2(p3[1]-p1[1],p3[0]-p1[0])
#     t2=np.arctan2(p2[1]-p1[1],p2[0]-p1[0])
#     tt=t2-t1
#     if tt>np.pi:t1+=2*np.pi
#     if tt<-np.pi:t2+=2*np.pi
#     tt=t2-t1
#     return tt

# def get_len(*args):
#     if len(args)==1:
#         p1=args[0][0]
#         p2=args[0][1]
#     elif len(args)==2:
#         p1=args[0]
#         p2=args[1]
#     else:
#         return None
#     return sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)


# def update_target_index(x,y):
#     dtheta=-cal_angle(trajs[now_target_index[0]],source,(x,y))
#     dis_thre=0.
#     dis_thre2=0.03
#     alpha=abs(cal_angle(trajs[last_target_index[0]],trajs[now_target_index[0]],source))
#     theta_thre=dis_thre*np.sin(alpha)/get_len(source,trajs[now_target_index[0]])
#     #print dtheta+theta_thre,get_len(trajs[now_target_index[0]],(x,y))
#     if dtheta+theta_thre>0 or get_len(trajs[now_target_index[0]],(x,y))<dis_thre2:
#         now_target_index[0]=now_target_index[0]+1
#         last_target_index[0]=last_target_index[0]+1
#         return True
#     else:
#         return False
#     pass


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

    return -kp*p-ki*i-kd*d,pid

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
    x=target[0]
    y=target[1]
    theta=target[2]

    a = -np.sin(theta)#line[0][1] - line[1][1]
    b = np.cos(theta)#line[1][0] - line[0][0]
    # c = line[0][0] *line[1][1] - line[1][0] * line[0][1]
    c= -a*x-b*y

    d=(a*x+b*y+c)/(a**2+b**2)

    return -d

    pass
    # if tt.isstright:
    #     a,b,c=getLinePara(lm)
    #     d=abs(a*x+b*y+c)/(a**2+b**2)
    #     if cal_angle(tl,(x,y),tt)>0:
    #         d=-d
    #     err=d
    # else:
    #     if tt.direction==1:
    #         err=-get_len(tt.rot_center,(x,y))+tt.rot_r
    #     else:
    #         #print tt.rot_r,get_len(tt.rot_center,(x,y))
    #         err=get_len(tt.rot_center,(x,y))-tt.rot_r
    # return err
    # pass
# def get_speed(tt,err,angle_err):
#     err_level=max(abs(err)-0.08,0.0)+max(abs(angle_err)-0.08,0.0)
#     if tt.speedlevel==0:
#         return max(40000-err_level/0.45*24000,16000.0)
#         # if err<0.08 and angle_err<0.08:
#         #     return 26000.0
#         # else:
#         #     return 16000.0
#     elif tt.speedlevel==1:
#         return max(25000.0-err_level/0.45*15000,10000.0)
#         # if err<0.12 and angle_err<0.08:
#         #     return 14000.0
#         # else:
#         #     return 9500.0
#     elif tt.speedlevel==2:
#         return 10000.0
#     elif tt.speedlevel==3:
#         return 7000.0
#     else:
#         return 6300.0

# def get_proposed_angle(x,y):
#     tt=trajs[now_target_index[0]]
#     tl=trajs[last_target_index[0]]
#     #print tt.rot_center
#     if tt.isstright:
#         predict_angle=np.arctan2(-tt[1]+tl[1],-tt[0]+tl[0])
#     else:
#         if tt.direction==1:
#             predict_angle=np.arctan2(-y+tt.rot_center[1],-x+tt.rot_center[0])-np.pi/2
#             #if predict_angle<-np.pi:predict_angle+=2*np.pi
#         else:
#             predict_angle=np.arctan2(-y+tt.rot_center[1],-x+tt.rot_center[0])+np.pi/2
#             #if predict_angle>np.pi:predict_angle-=2*np.pi
#     return predict_angle

def get_error_angle(x,y,yaw,target):
    theta=target[2]

    err=-pi_2_pi(yaw-theta)
    return err
    #print now_target_index[0]
    # tt=trajs[now_target_index[0]]
    # tl=trajs[last_target_index[0]]
    # #print tt.rot_center
    # if tt.isstright:
    #     # a,b,c=getLinePara([tt,tl])
    #     # d=abs(a*x+b*y+c)/(a**2+b**2)
    #     # if cal_angle((x,y),tl,tl)<0:
    #     #     d=-d
    #     predict_angle=np.arctan2(-tt[1]+tl[1],-tt[0]+tl[0])
    #     err=predict_angle-angle
    #     if err<np.pi:err+=2*np.pi
    #     if err>np.pi:err-=2*np.pi
    # else:
    #     if tt.direction==1:
    #         predict_angle=np.arctan2(-y+tt.rot_center[1],-x+tt.rot_center[0])-np.pi/2
    #         #if predict_angle<-np.pi:predict_angle+=2*np.pi
    #         err=predict_angle-angle
    #         if err<np.pi:err+=2*np.pi
    #         if err>np.pi:err-=2*np.pi
    #     else:
    #         predict_angle=np.arctan2(-y+tt.rot_center[1],-x+tt.rot_center[0])+np.pi/2
    #         #if predict_angle>np.pi:predict_angle-=2*np.pi
    #         err=predict_angle-angle
    #         if err<np.pi:err+=2*np.pi
    #         if err>np.pi:err-=2*np.pi
    #     #err=get_len(tt.rot_center,(x,y))-tt.rot_r
    # return err
    pass


if __name__ == '__main__':

    target=[2,2,np.pi/4-np.pi]

    print get_error(1,1,target)

    print np.rad2deg(get_error_angle(1,1,np.deg2rad(30),target))
    # init()
    # last_target_index=[4]
    # now_target_index=[5]
    # #print cal_angle((np.sqrt(3)/2.,-1/2.0),(0,0),(np.sqrt(3)/2.,1/2.0))/np.pi*180
    # print now_target_index[0]
    # x=0.6;y=2.4848
    # angle=1.0
    # print update_target_index(x,y)
    # print now_target_index[0]
    # tt=trajs[now_target_index[0]]
    # print tt.direction
    # err=get_error(x,y)
    # angle_err=get_error_angle(angle,x,y)
    # print err,angle_err
    # print PID(err,angle_err,tt.servo,[0.01])
    #self.previous_err.appedn(err)
    #print PID()
