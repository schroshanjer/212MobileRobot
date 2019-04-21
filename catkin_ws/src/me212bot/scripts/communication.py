"""

Steps to communicate with Mobile Robot

1. Connect to Wifi "212IntroRobot", password is : robot2016

2. Run the following command in Terminal:

ssh-keygen
ssh-copy-id -i ~/.ssh/id_rsa.pub robot@192.168.0.63

(password is robot2016)

3. use 'ssh 192.168.0.63' to test ssh connection

4. import this file and call function Call_Mobile_Robot(cmd)

This is an expedience, I will try to improve it if necessary.

-Shangjie

"""

import os,time

def Call_Mobile_Robot(msg='test'):

    filename='msg_from_delta_%.3f.txt'%time.time()
    file=os.path.join('.',filename)

    with open(file,'w') as f:
        f.write(msg)

    cmd='scp %s robot@192.168.0.63:~/msg/'%file

    os.system(cmd)

    os.remove(file)
    pass

if __name__ == '__main__':
    Call_Mobile_Robot('test')