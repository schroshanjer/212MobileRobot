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

def main():
    rospy.sleep(1)
    thread = threading.Thread(target = send_vel_loop)
    thread.start()
    rospy.spin()

def send_vel_loop():
    velcmd_pub = rospy.Publisher('/cmdvel', WheelCmdVel, queue_size = 1)
    rate = rospy.Rate(100) # 100hz
    
    while not rospy.is_shutdown() :
        wcv = WheelCmdVel()
        cmd=raw_input()
        if cmd=='w':
            wcv.desiredWV_R = 0.1
            wcv.desiredWV_L = 0.1
        elif cmd=='a':
            wcv.desiredWV_R = 0.2
            wcv.desiredWV_L = 0.1
        elif cmd=='d':
            wcv.desiredWV_R = 0.1
            wcv.desiredWV_L = 0.2
        elif cmd=='s':
            wcv.desiredWV_R = -0.1
            wcv.desiredWV_L = -0.1
        else:
            wcv.desiredWV_R = 0.
            wcv.desiredWV_L = 0.

        
        velcmd_pub.publish(wcv) 
        
        rate.sleep() 

if __name__=='__main__':
    main()