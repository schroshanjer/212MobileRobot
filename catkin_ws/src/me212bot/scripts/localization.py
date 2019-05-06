#!/usr/bin/python

import rospy
import threading

import tf.transformations as tfm
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from std_msgs.msg import Header


import numpy as np

import traceback,time

from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad, poselist2pose
from me212bot.msg import WheelCmdVel,WheelEncoder

rospy.init_node('localization',anonymous=True)

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

class State(object):
    """docstring for State"""
    def __init__(self):
        super(State, self).__init__()
        #self.arg = arg

        self.encoder_data=[]

        self.tag_num=2
        self.tag_pose=np.array([0,0,-np.pi])
        self.tag_data=[None]*self.tag_num


state=State()


def april_tag_callback(data,tag_id):

    state.tag_data[tag_id]=data
    pass

def encoder_callback(data):

    state.encoder_data.append(data)
    pass

def Localization():
    while True:
        rospy.sleep(1.)
        print state.encoder_data
        print state.tag_data
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




        
