"""

Extended kalman filter (EKF) localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import math
import matplotlib.pyplot as plt

# Estimation parameter of EKF
Q0 = np.array([0.1, 0.1, 0.2, 1.0])**2  # predict state covariance
R0 = np.array([0.1, np.deg2rad(5.), np.deg2rad(5.0)])**2  # Observation x, theta_robot,theta_atag position covariance

#  Simulation parameter
Qsim = np.diag([1.0, np.deg2rad(30.0)])**2
Rsim = np.diag([0.5, 0.5])**2

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]

show_animation = True

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def calc_input():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]
    u = np.array([[v, yawrate]]).T
    return u


def observation(xTrue, xd, u):

    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    zx = xTrue[0, 0] + np.random.randn() * Rsim[0, 0]
    zy = xTrue[1, 0] + np.random.randn() * Rsim[1, 1]
    z = np.array([[zx, zy]]).T

    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * Qsim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * Qsim[1, 1]
    ud = np.array([[ud1, ud2]]).T

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


def motion_model(x, u,DT):

    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F.dot(x) + B.dot(u)
    
    #print 'Bu',x

    return x


def observation_model(x):
    #  Observation Model
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0,0,1,0]
    ])

    z = H.dot(x)

    return z


def jacobF(x, u,DT):
    """
    Jacobian of Motion Model

    motion model
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    so
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    """
    yaw = x[2, 0]
    v = u[0, 0]
    jF = np.array([
        [1.0, 0.0, -DT * v * math.sin(yaw), 0],#DT * math.cos(yaw)
        [0.0, 1.0, DT * v * math.cos(yaw), 0],#DT * math.sin(yaw)
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    return jF


def jacobH(x):
    # Jacobian of Observation Model
    jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0,0,1,0]
    ])

    return jH

def cal_R(z,lm):
    dx=lm[0]-z[0,0]
    dy=lm[1]-z[1,0]

    r=np.sqrt(dx**2+dy**2)

    R_1=np.diag([R0[0]*r**2, R0[1]*r**2])

    eigen_r=np.array([dx,dy])/r

    eigen_theta=np.array([dx,-dy])/r

    S=np.array([eigen_r,eigen_theta])

    R2=(S.dot(R_1)).dot(S.T)

    R=np.zeros((3,3))
    R[0:2,0:2]=R2
    R[2,2]=R0[2]

    return R

def cal_Q(x,u,dt):
    Q=np.zeros((4,4))
    if np.linalg.norm(u)==0 or dt==0:
        return Q
    x_err=Q0[0]*abs(u[0,0])*dt
    y_err=Q0[0]*abs(u[0,0])*np.sin(u[1,0]*dt)*dt
    #print x_err,y_err
    theta_err=Q0[2]*(abs(u[1,0])*dt)
    
    yaw = x[2, 0]

    Q2=np.diag([x_err,y_err])

    #print Q2

    #print Q2
    
    H=np.array([[np.cos(yaw),np.sin(yaw)],
                [-np.sin(yaw),np.cos(yaw)]])

    Q[0:2,0:2]=H.dot(Q2).dot(H.T)
    Q[2,2]=theta_err

    return Q
    
    pass




def ekf_estimation(xEst, PEst, z):

    #  Predict
    xPred = motion_model(xEst, u)
    jF = jacobF(xPred, u)
    PPred = jF.dot(PEst).dot(jF.T) + Q

    #  Update
    jH = jacobH(xPred)
    zPred = observation_model(xPred)
    y = z - zPred
    S = jH.dot(PPred).dot(jH.T) + R
    K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    xEst = xPred + K.dot(y)
    PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)

    return xEst, PEst

def ekf_update(xEst, PEst, zs, u,lm,dt):

    #  Predict
    xPred = motion_model(xEst, u,dt)
    #xPred=xEst
    jF = jacobF(xPred, u, dt)
    PPred = jF.dot(PEst).dot(jF.T) + cal_Q(xEst,u,DT)
    #print jF.dot(PEst).dot(jF.T)
    #print cal_Q(xEst,u,DT)
    if not zs:
        xPred[2,0]=pi_2_pi(xPred[2,0])
        return xPred,PPred
    for z in zs:
        
        #print z
        lm_id=int(z[3,0])
        z=z[0:3,:]

        #  Update
        xPred_z=np.vstack([z,[0]])
        
        #jH = jacobH(xPred)
        jH = jacobH(xPred_z)
        zPred = observation_model(xPred)
        #print(zPred.shape,z.shape)
        y = z - zPred
        S = jH.dot(PPred).dot(jH.T) + cal_R(z,lm[lm_id,:])
        K = PPred.dot(jH.T).dot(np.linalg.inv(S))
        
        #xEst = xPred + K.dot(y)
        xEst = xPred_z + K.dot(-y)
        
        xEst[2,0]=pi_2_pi(xEst[2,0])
        
        PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)
        print z
        print xEst

    return xEst, PEst


def plot_covariance_ellipse(xEst, PEst):  # pragma: no cover
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    R = np.array([[math.cos(angle), math.sin(angle)],
                  [-math.sin(angle), math.cos(angle)]])
    fx = R.dot(np.array([x, y]))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--")


def main2():
    print(__file__ + " start!!")

    time = 0.0

    # State Vector [x y yaw v]'
    xEst = np.zeros((4, 1))
    xTrue = np.zeros((4, 1))
    PEst = np.eye(4)

    xDR = np.zeros((4, 1))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hz = np.zeros((2, 1))

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u)

        xEst, PEst = ekf_estimation(xEst, PEst, z, ud)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.hstack((hz, z))

        if show_animation:
            plt.cla()
            plt.plot(hz[0, :], hz[1, :], ".g")
            plt.plot(hxTrue[0, :].flatten(),
                     hxTrue[1, :].flatten(), "-b")
            plt.plot(hxDR[0, :].flatten(),
                     hxDR[1, :].flatten(), "-k")
            plt.plot(hxEst[0, :].flatten(),
                     hxEst[1, :].flatten(), "-r")
            plot_covariance_ellipse(xEst, PEst)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    #main()
    plt.cla()
    xEst=np.array([[0,0,1.7,0]]).T
    PEst = np.eye(4)

    z=[np.array([0.1,0.2,0.1,0]).reshape(-1,1),np.array([0.1,0.2,0.1,1]).reshape(-1,1)]
    lm=np.array([[2,3,1.7],
                  [-1,2,1.7],
        ])
    #PEst=cal_R(xEst[:,0].T,lm)
    #plot_covariance_ellipse(xEst, PEst)
    u=np.array([[1,1]]).T
    #xEst, PEst=ekf_update(xEst, PEst, z,u,lm,DT)

    PEst=np.diag([3,1])
    yaw=np.pi/6
    H=np.array([[np.cos(yaw),-np.sin(yaw)],
        [np.sin(yaw),np.cos(yaw)]])
    PEst=H.dot(PEst).dot(H.T)
    plot_covariance_ellipse(xEst, PEst)

    plt.show()
