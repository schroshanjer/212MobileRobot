"""

Steps to communicate with Mobile Robot

1. Connect to Wifi "212IntroRobot", password is : robot2016

2. Run the following command in Terminal:

ssh-keygen
ssh-copy-id -i ~/.ssh/id_rsa.pub robot@192.168.0.63

(password is robot2016)

3. use 'ssh 192.168.0.63' to test ssh connection

4. import this file and call function Call_Mobile_Pizza_ready() or Call_Mobile_Pizza_pushed()

-Shangjie

"""

import os,time
import subprocess

MOBILE_IP='192.168.0.63'
DELTA_IP='192.168.0.64'

def Call_Mobile_Pizza_ready():
    for i in range(10):
        result=sent_param(MOBILE_IP,'PIZZA_READY',1)
        if result:
            return True
        time.sleep(0.1)
    print 'PIZZA READY COMMAND SENT FAILED'
    return False

def Call_Mobile_Pizza_pushed():
    for i in range(10):
        result=sent_param(MOBILE_IP,'PIZZA_PUSHED',1)
        if result:
            return True
        time.sleep(0.1)
    print 'PIZZA PUSHED COMMAND SENT FAILED'
    return False

def Call_Delta_Mobile_arrived():
    for i in range(10):
        result=sent_param(DELTA_IP,'MOBILE_ARRIVED',1)
        if result:
            return True
        time.sleep(0.1)
    print 'MOBILE ARRIVED COMMAND SENT FAILED'
    return False
    pass

def Call_Delta_Mobile_left():
    for i in range(10):
        result=sent_param(DELTA_IP,'MOBILE_ARRIVED',0)
        if result:
            return True
        time.sleep(0.1)
    print 'MOBILE LEFT COMMAND SENT FAILED'
    return False
    pass

def sent_param(ip,param,value):
    cmd1='export ROS_MASTER_URI=http://%s:11311'%ip
    cmd2='rosparam set %s '%param+str(value)
    process = subprocess.Popen("/bin/bash", shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE);
    process.stdin.write("cmd1\n")
    process.stdin.write("cmd2\n")
    process.stdin.flush()
    stdout, stderr = process.communicate()
    return not bool(stderr)
    pass

def Call_Mobile_Robot(msg='test'):
    '''
    This is the old version, Don't use this
    '''

    filename='msg_from_delta_%.3f.txt'%time.time()
    file=os.path.join('.',filename)

    with open(file,'w') as f:
        f.write(msg)

    cmd='scp %s robot@192.168.0.63:~/msg/'%file

    os.system(cmd)

    os.remove(file)
    pass

if __name__ == '__main__':
    #Call_Mobile_Robot('test')
    Call_Mobile_Pizza_ready()
    time.sleep(1)
    Call_Mobile_Pizza_pushed()