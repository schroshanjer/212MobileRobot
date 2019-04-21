import numpy as np

import time,os

import cPickle as pickle

# servo value (0 to 1) =  steering_angle_to_servo_gain * steering angle (radians) + steering_angle_to_servo_offset

steering_angle_to_servo_gain=0.8942
steering_angle_to_servo_offset=0.5201

robot_b=0.225
wheel_r=0.037

robot_aw=-0.12
robot_uaw=robot_aw+0.32

max_detect_range=75
data_dir='~/pkl_files'

def save_data(laser,result):
    #datetime.datetime.now().strftime('%Y-%m-%d')
    filename=str(time.time())+'.pkl'
    with open(os.path.join(data_dir,filename), 'wb') as f:
        pickle.dump((laser,result),f)
    return



def servo_to_rad(servo):
    #rad=(servo-0.5)/0.35*np.pi/3
    rad=(servo-steering_angle_to_servo_offset)/steering_angle_to_servo_gain
    return rad

def rad_to_servo(rad):
    servo=steering_angle_to_servo_gain*rad+steering_angle_to_servo_offset
    #servo=0.5+rad/(np.pi/3)*0.35
    return servo

def speed_in_meter(speed):
    return speed/4614.0

def ranges_in_meter(ranges):
    return ranges

def angles_in_caculating_frame(angles):
    angles=angles+np.pi
    angles=np.where(angles>np.pi,angles-2*np.pi,angles)
    return -angles

def get_rot_center(alpha):
    #alpha is the angle between front wheel and front (clockwise is positive, 0 is stright forward)
    if abs(alpha) < 0.01:
        return None
    
    #robot_uaw=0.04
    #robot_aw=-0.28
    fw=robot_uaw
    bw=robot_aw

    x=-(bw-fw)/np.tan(alpha)
    y=bw
    return (x,y)

def r_to_alpha(r):
    fw=robot_uaw
    bw=robot_aw

    return np.arctan((fw-bw)/r)

def alpha_to_w(alpha,robotVel):
    #alpha is the angle between front wheel and front (clockwise is positive, 0 is stright forward)
    

    fw=robot_uaw
    bw=robot_aw

    K=np.tan(alpha)/(bw-fw)
    
    # hh=2*v/wheel_r
    # cc=k*robot_b*hh
    # wr=(hh+cc)/2
    # wl=(hh-cc)/2
    desiredWV_L = robotVel - K *robot_b * robotVel;
    desiredWV_R = 2*robotVel - desiredWV_L ;

    return desiredWV_R,desiredWV_L


def find_direction(laser_msgs, margin=0.5,stop_margin=1.0):
    #speed=speed_in_meter(speed)
    #alpha=servo_to_rad(servo)
    ranges=ranges_in_meter(np.array(laser_msgs.ranges))
    intensity=np.array(laser_msgs.intensities)

    angles=np.linspace(laser_msgs.angle_min,laser_msgs.angle_max,len(ranges))
    angles=angles_in_caculating_frame(angles)

    valid_indx=np.where(np.abs(angles)<=np.pi/2)
    print np.transpose([angles[valid_indx],ranges[valid_indx],intensity[valid_indx]])

    angles=angles[valid_indx]
    ranges=ranges[valid_indx]
    intensity=intensity[valid_indx]
    valid_indx=np.where(intensity>0)
    angles=angles[valid_indx]
    ranges=ranges[valid_indx]
    intensity=intensity[valid_indx]
    obstacle=np.transpose([angles,ranges])


    angle_list=np.linspace(-r_to_alpha(robot_b*2),r_to_alpha(robot_b*2),25)
    #print angle_list

    distance_list=[]
    for angle in angle_list:
        
        distance= get_distance(angle,obstacle,margin)
        distance_list.append(distance)

    if np.max(distance_list)<=stop_margin-margin:
        debug=np.array([angle_list,distance_list])
        debug=debug.transpose()
        return None,debug
    
    direction=angle_list[np.where(distance_list==np.max(distance_list))]
    if type(direction) is not type(angle_list[0]):
        direction=direction[np.where(np.abs(direction)==np.min(np.abs(direction)))]
        if type(direction) is not type(angle_list[0]):
            direction=direction[0]
    debug=np.array([angle_list,distance_list])
    debug=debug.transpose()#.reshape(-1)
    
    return direction,debug




def get_distance(angle, obstacle, margin=0.5, noise_level=2):
    obstacle=np.array(obstacle)
    beta=obstacle[:,0]
    d=obstacle[:,1]
    
    

    center=get_rot_center(angle)
    if not center:
        
        x=d*np.sin(beta)
        y=d*np.cos(beta)
        crossing=np.where(np.abs(x)>margin,0,1)
        collide_indx=np.where(crossing==1)
        if len(collide_indx[0])==0:
            return max_detect_range
        x=np.where(np.abs(x)<=margin,x,margin)
        distance=y-np.sqrt(margin*margin-x*x)
        distance_list=sorted(distance[collide_indx])
        #return np.min(distance[collide_indx])
        if len(distance_list)<=noise_level:
            return max_detect_range
        return distance_list[noise_level]
        #distance_t2=distance-speed*timestep
        #collide=np.where(distance_t2<=0,1,0)*crossing
        #print collide
        # if collide.sum()>=noise_level:
        #     return True,crossing
        # else:
        #     return False,[]



    x0,y0=center
    r0=np.sqrt(x0**2+y0**2)

    x=-x0+d*np.sin(beta)
    y=-y0+d*np.cos(beta)
    
    max_go_range=(np.pi-np.abs(np.arctan(y0/x0)))*r0
    #print 'max_go_range',max_go_range,'center',center

    #direction=np.sign(angle)
    if angle>0:
    
        rho0=np.sqrt(x*x+y*y)
        theta_obstacle=np.arctan(y/x)
        theta_robot=np.arctan(y0/x0)
        theta_obstacle=np.where(theta_obstacle<0,theta_obstacle+np.pi,theta_obstacle)
        if theta_robot<0:theta_robot+=np.pi

        cos_dtheta=(rho0*rho0+r0**2-margin**2)/2/rho0/r0

        crossing=np.where(np.abs(cos_dtheta)<=1,1,0)
        collide_indx=np.where(crossing==1)

        if len(collide_indx[0])==0:
            return min(max_detect_range,max_go_range)
        cos_dtheta=np.where(np.abs(cos_dtheta)<=1,cos_dtheta,1)
        dtheta=np.arccos(cos_dtheta)

        theta_crossing=theta_robot-dtheta


        distance=(theta_crossing-theta_obstacle)*r0
    else:
        rho0=np.sqrt(x*x+y*y)
        theta_obstacle=np.arctan(y/x)
        theta_robot=np.arctan(y0/x0)
        theta_obstacle=np.where(theta_obstacle<0,theta_obstacle+np.pi,theta_obstacle)
        if theta_robot<0:theta_robot+=np.pi

        cos_dtheta=(rho0*rho0+r0**2-margin**2)/2/rho0/r0

        crossing=np.where(np.abs(cos_dtheta)<=1,1,0)
        collide_indx=np.where(crossing==1)

        if len(collide_indx[0])==0:
            return min(max_detect_range,max_go_range)
        cos_dtheta=np.where(np.abs(cos_dtheta)<=1,cos_dtheta,1)
        dtheta=np.arccos(cos_dtheta)

        theta_crossing=theta_robot+dtheta


        distance=-(theta_crossing-theta_obstacle)*r0
    
    distance_list=sorted(distance[collide_indx])
    return min(distance_list[min(noise_level,len(distance_list)-1)],max_go_range)


    # if collide.sum()>=noise_level:
    #     return True,crossing
    # else:
    #     return False,[]
    # pass


if __name__ == '__main__':
    print get_rot_center(3.14/6)
    obstacle=[[0.,1.],[0.8,0.8]]
    print check_collision(5.,-1.14/6,obstacle)