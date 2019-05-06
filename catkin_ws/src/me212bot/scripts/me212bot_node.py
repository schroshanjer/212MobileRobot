#!/usr/bin/python

# 2.12 Lab 3 me212bot_node: ROS driver running on the pc side to read and send messages to Arduino
# Peter Yu Sept 2016

import rospy
import threading
import serial
import tf.transformations as tfm
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from std_msgs.msg import Header

import traceback

import helper
from me212bot.msg import WheelCmdVel,WheelEncoder

serialComm = serial.Serial('/dev/ttyACM0', 115200, timeout = 5)

odom_pub = rospy.Publisher("/odom_pose", PoseStamped, queue_size = 1)

encoder_pub = rospy.Publisher("/odom_encoder", PoseStamped, queue_size = 1)
## main function (Need to modify)
def main():
    rospy.init_node('me212bot', anonymous=True)
    
    odometry_thread = threading.Thread(target = read_odometry_loop)
    odometry_thread.start()
    
    ## 1. Initialize a subscriber
    rospy.Subscriber('/cmdvel', WheelCmdVel, cmdvel_callback)
    
    rospy.spin()


## msg handling function (Need to modify)
def cmdvel_callback(msg):  
    ## 2. Send msg.desiredWV_R and msg.desiredWV_L to Arduino.
    strCmd =  str(msg.desiredWV_R) + ',' + str(msg.desiredWV_L) + '\n'
    serialComm.write(strCmd)
    


# read_odometry_loop() is for reading odometry from Arduino and publish to rostopic. (No need to modify)
def read_odometry_loop():
    prevtime = rospy.Time.now()
    while not rospy.is_shutdown():
        # get a line of string that represent current odometry from serial
        serialData = serialComm.readline()
        #print serialData
        pose_stamp=PoseStamped()
        pose_stamp.header.stamp = prevtime
        pose_stamp.header.frame_id = "/map"
        # split the string e.g. "0.1,0.2,0.1" with cammas
        splitData = serialData.split(',')

        wheelencode=WheelEncoder()
        wheelencode.Timestamp=prevtime.to_sec()
        pose_stamp.header.stamp = rospy.Time.now()
        
        # parse the 3 split strings into 3 floats
        try:
            x     = float(splitData[0])
            y     = float(splitData[1])
            theta = float(splitData[2])

            dTL=float(splitData[3])
            dTR=float(splitData[4])

            distance=float(splitData[5])
            
            hz    = 1.0 / (rospy.Time.now().to_sec() - prevtime.to_sec())
            #prevtime = rospy.Time.now()

            wheelencode.Theta_R=dTR
            wheelencode.Theta_L=dTL
            
            print 'TL=', dTL, 'TR=', dTR, 'PD=', distance
            #print 'x=', x, ' y=', y, ' theta =', theta, ' hz =', hz
            
            # publish odometry as Pose msg
            odom = Pose()
            odom.position.x = x
            odom.position.y = y
            
            qtuple = tfm.quaternion_from_euler(0, 0, theta)
            odom.orientation = Quaternion(qtuple[0], qtuple[1], qtuple[2], qtuple[3])

            pose_stamp.pose=odom

            odom_pub.publish(pose_stamp)

            encoder_pub.publish(wheelencode)

            
        except Exception:
            #pass
            #print traceback.print_exc()
            # print out msg if there is an error parsing a serial msg
            print 'Cannot parse', splitData
            

if __name__=='__main__':
    main()


