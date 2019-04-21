"""

FastSLAM 1.0 example

author: Atsushi Sakai (@Atsushi_twi)

Modified by: @sjxue

"""

import numpy as np
import math
import matplotlib.pyplot as plt
import sys

sys.path.append('../')

import pickle as pkl

#from simulation import *

#from predict import *


# Fast SLAM covariance
Q = np.diag([3.0, np.deg2rad(10.0)])**2
Q_rad=np.diag([9900.0, np.deg2rad(360./40*2)])**2
R = np.diag([1.0, np.deg2rad(20.0)])**2

#  Simulation parameter
Qsim = np.diag([0.3, np.deg2rad(2.0)])**2
Qsim_rad = np.diag([25.0, np.deg2rad(360./40/2.)])**2
Rsim = np.diag([0.5, np.deg2rad(10.0)])**2
OFFSET_YAWRATE_NOISE = 0.01

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 200.0  # maximum observation range
M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM srate size [x,y]
N_PARTICLE = 1200  # number of particle
NTH = N_PARTICLE / 1.5  # Number of particle for re-sampling

show_animation = True
record_data=True
recordpath = '../../../data/drd/slam_0303_2'
if __name__ == '__main__' and record_data:
    if not os.path.isdir(recordpath):
        os.mkdir(recordpath)


class Particle:

    def __init__(self, N_LM):
        self.w = 1.0 / N_PARTICLE
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        # landmark x-y positions
        self.lm = np.zeros((N_LM, LM_SIZE))
        # landmark position covariance
        self.lmP = np.zeros((N_LM * LM_SIZE, LM_SIZE))

        self.lm_rad=np.zeros((1,LM_SIZE))
        self.lm_rad_init=np.array(np.zeros((1,1)),dtype=bool)
        #print(self.lm_rad_init[0,0])
        self.lmP_rad = np.zeros((1 * LM_SIZE, LM_SIZE))


def fast_slam1(particles, u, z, z_rad):

    particles = predict_particles(particles, u)

    particles = update_with_observation(particles, z, z_rad)

    particles = resampling(particles)

    return particles


def normalize_weight(particles):

    sumw = sum([p.w for p in particles])

    try:
        for i in range(N_PARTICLE):
            particles[i].w /= sumw
    except ZeroDivisionError:
        for i in range(N_PARTICLE):
            particles[i].w = 1.0 / N_PARTICLE

        return particles

    return particles


def calc_final_state(particles):

    xEst = np.zeros((STATE_SIZE, 1))

    particles = normalize_weight(particles)

    for i in range(N_PARTICLE):
        xEst[0, 0] += particles[i].w * particles[i].x
        xEst[1, 0] += particles[i].w * particles[i].y
        xEst[2, 0] += particles[i].w * particles[i].yaw

    xEst[2, 0] = pi_2_pi(xEst[2, 0])
    #  print(xEst)

    return xEst


def predict_particles(particles, u):

    for i in range(N_PARTICLE):
        px = np.zeros((STATE_SIZE, 1))
        px[0, 0] = particles[i].x
        px[1, 0] = particles[i].y
        px[2, 0] = particles[i].yaw
        ud = u + (np.random.randn(1, 2) @ R).T  # add noise
        px = motion_model(px, ud)
        particles[i].x = px[0, 0]
        particles[i].y = px[1, 0]
        particles[i].yaw = px[2, 0]

    return particles


def add_new_lm(particle, z, Q):

    r = z[0]
    b = z[1]
    lm_id = int(z[2])

    s = math.sin(pi_2_pi(particle.yaw + b))
    c = math.cos(pi_2_pi(particle.yaw + b))

    particle.lm[lm_id, 0] = particle.x + r * c
    particle.lm[lm_id, 1] = particle.y + r * s

    # covariance
    Gz = np.array([[c, -r * s],
                   [s, r * c]])

    particle.lmP[2 * lm_id:2 * lm_id + 2] = Gz @ Q @ Gz.T

    return particle

def add_new_lm_rad(particle, z, Q):

    r = z[0]
    r=np.sqrt(Qsim_rad[0, 0])*np.random.randn()
    b = z[1]
    lm_id = int(z[2])

    s = math.sin(pi_2_pi(particle.yaw + b))
    c = math.cos(pi_2_pi(particle.yaw + b))

    particle.lm_rad[lm_id, 0] = particle.x + r * c
    particle.lm_rad[lm_id, 1] = particle.y + r * s
    #print(particle.x,particle.y,r,b,particle.lm_rad[lm_id,:])
    # covariance
    Gz = np.array([[c, -r * s],
                   [s, r * c]])

    particle.lmP_rad[2 * lm_id:2 * lm_id + 2] = Gz @ Q @ Gz.T

    return particle

def compute_jacobians(particle, xf, Pf, Q):
    dx = xf[0, 0] - particle.x
    dy = xf[1, 0] - particle.y
    d2 = dx**2 + dy**2
    d = math.sqrt(d2)

    zp = np.array(
        [d, pi_2_pi(math.atan2(dy, dx) - particle.yaw)]).reshape(2, 1)

    Hv = np.array([[-dx / d, -dy / d, 0.0],
                   [dy / d2, -dx / d2, -1.0]])

    Hf = np.array([[dx / d, dy / d],
                   [-dy / d2, dx / d2]])

    Sf = Hf @ Pf @ Hf.T + Q

    return zp, Hv, Hf, Sf

def compute_jacobians_rad(particle, xf, Pf, Q):
    dx = xf[0, 0] - particle.x
    dy = xf[1, 0] - particle.y
    d2 = dx**2 + dy**2
    d = math.sqrt(d2)

    zp = np.array(
        [d, pi_2_pi(math.atan2(dy, dx) - particle.yaw)]).reshape(2, 1)

    Hv = np.array([[-dx / d, -dy / d, 0.0],
                   [dy / d2, -dx / d2, -1.0]])

    Hf = np.array([[dx / d, dy / d],
                   [-dy / d2, dx / d2]])

    Sf = Hf @ Pf @ Hf.T + Q

    return zp, Hv, Hf, Sf


def update_KF_with_cholesky(xf, Pf, v, Q, Hf):
    PHt = Pf @ Hf.T
    S = Hf @ PHt + Q

    S = (S + S.T) * 0.5
    SChol = np.linalg.cholesky(S).T
    SCholInv = np.linalg.inv(SChol)
    W1 = PHt @ SCholInv
    W = W1 @ SCholInv.T

    x = xf + W @ v
    P = Pf - W1 @ W1.T

    return x, P

def update_KF_with_cholesky_rad(xf, Pf, v, Q, Hf):
    PHt = Pf @ Hf.T
    S = Hf @ PHt + Q

    S = (S + S.T) * 0.5
    SChol = np.linalg.cholesky(S).T
    SCholInv = np.linalg.inv(SChol)
    W1 = PHt @ SCholInv
    W = W1 @ SCholInv.T

    x = xf + W @ v
    P = Pf - W1 @ W1.T
    #print(dz)
    # print(x)
    # print(P)
    return x, P

def update_KF_rad2(xf, Pf, v, Q, Hf):
    PHt = Pf @ Hf.T
    P = Hf @ PHt

    K=PHt@np.linalg.inv (P+Q)


    x = xf + K@(v)
    Pf = Pf - K @ Hf @Pf
    #print(dz)
    #print(x)
    
    #Pf = np.linalg.inv(Hf) @ P @ np.linalg.inv(Hf.T)
    #print(Pf)
    return x, Pf

def update_KF_rad(xf, Pf, v, Q, Hf):
    #print ('xf',xf)
    #print('P_begin',Pf)
    PHt = Pf @ Hf.T
    P = Hf @ PHt

    
    c = Q[1,1]
    d = P[0,0]
    e = P[0,1]
    f = P[1,1]

    # P = np.array([[(c*d-e**2+d*f)/(c+f),c*e/(c+f)],
    #              [c*e/(c+f),c*f/(c+f)]])
    K=PHt@np.array([[0,0],[0,1/(c+f)]])
    #P= P - np.array([[e**2/(c+f),f*e/(c+f)],
    #             [f*e/(c+f),f**2/(c+f)]])
    # dtt = v[1,0]
    # dz = np.array([e/(c+f)*dtt,f/(c+f)*dtt]).reshape(2,1)
    x = xf + K@(v)
    Pf = Pf - K @ Hf @Pf
    # #print()
    # x = xf + Hf.T @ dz

    # print('P',P)
    # Pf = np.linalg.inv(Hf) @ P @ np.linalg.inv(Hf.T)

    # print(Hf)
    # print(x)
    # print(Pf)

    # S = (S + S.T) * 0.5
    # SChol = np.linalg.cholesky(S).T
    # SCholInv = np.linalg.inv(SChol)
    # W1 = PHt @ SCholInv
    # W = W1 @ SCholInv.T

    # x = xf + W @ v
    # P = Pf - W1 @ W1.T

    return x, Pf


def update_landmark(particle, z, Q):

    lm_id = int(z[2])
    xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)
    Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2, :])

    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q)

    dz = z[0:2].reshape(2, 1) - zp
    dz[1, 0] = pi_2_pi(dz[1, 0])

    xf, Pf = update_KF_with_cholesky(xf, Pf, dz, Q, Hf)

    particle.lm[lm_id, :] = xf.T
    particle.lmP[2 * lm_id:2 * lm_id + 2, :] = Pf

    return particle

def update_landmark_rad(particle, z, Q):

    lm_id = int(z[2])
    xf = np.array(particle.lm_rad[lm_id, :]).reshape(2, 1)
    Pf = np.array(particle.lmP_rad[2 * lm_id:2 * lm_id + 2, :])

    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q)

    dz = z[0:2].reshape(2, 1) - zp
    dz[1, 0] = pi_2_pi(dz[1, 0])

    xf, Pf = update_KF_rad(xf, Pf, dz, Q, Hf)



    particle.lm_rad[lm_id, :] = xf.T
    particle.lmP_rad[2 * lm_id:2 * lm_id + 2, :] = Pf

    return particle


def compute_weight(particle, z, Q):
    lm_id = int(z[2])
    xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)
    Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2])
    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q)

    dx = z[0:2].reshape(2, 1) - zp
    dx[1, 0] = pi_2_pi(dx[1, 0])

    try:
        invS = np.linalg.inv(Sf)
    except np.linalg.linalg.LinAlgError:
        print("singuler")
        return 1.0

    num = math.exp(-0.5 * dx.T @ invS @ dx)
    den = 2.0 * math.pi * math.sqrt(np.linalg.det(Sf))

    w = num / den

    return w

def compute_weight_rad(particle, z, Q):
    lm_id = int(z[2])
    xf = np.array(particle.lm_rad[lm_id, :]).reshape(2, 1)
    Pf = np.array(particle.lmP_rad[2 * lm_id:2 * lm_id + 2])
    #zp, Hv, Hf, Sf = compute_jacobians_rad(particle, xf, Pf, Q)

    dx = xf[0, 0] - particle.x
    dy = xf[1, 0] - particle.y
    d2 = dx**2 + dy**2
    d = math.sqrt(d2)

    zp = np.array(
        [d, pi_2_pi(math.atan2(dy, dx) - particle.yaw)]).reshape(2, 1)

    c=Q[1,1]


    dx = z[0:2].reshape(2, 1) - zp
    tt = pi_2_pi(dx[1, 0])

    w=math.exp(-0.5*(tt**2)/c)/(2*math.pi*math.sqrt(c))
    #print('weight',w)

    # try:
    #     invS = np.linalg.inv(Sf)
    # except np.linalg.linalg.LinAlgError:
    #     print("singuler")
    #     return 1.0

    # num = math.exp(-0.5 * dx.T @ invS @ dx)
    # den = 2.0 * math.pi * math.sqrt(np.linalg.det(Sf))

    # w = num / den

    return w


def update_with_observation(particles, z, z_rad):

    for iz in range(len(z[0, :])):

        lmid = int(z[2, iz])

        for ip in range(N_PARTICLE):
            # new landmark
            if abs(particles[ip].lm[lmid, 0]) <= 0.01:
                particles[ip] = add_new_lm(particles[ip], z[:, iz], Q)
            # known landmark
            else:
                w = compute_weight(particles[ip], z[:, iz], Q)
                particles[ip].w *= w
                particles[ip] = update_landmark(particles[ip], z[:, iz], Q)

    for iz in range(len(z_rad[0, :])):

        lmid = int(z_rad[2, iz])

        for ip in range(N_PARTICLE):
            # new landmark
            if not particles[ip].lm_rad_init[lmid, 0]:
                particles[ip] = add_new_lm_rad(particles[ip], z_rad[:, iz], Q_rad)
                particles[ip].lm_rad_init[lmid, 0]=True
            # known landmark
            else:
                w = compute_weight_rad(particles[ip], z_rad[:, iz], Q_rad)
                particles[ip].w *= w
                particles[ip] = update_landmark_rad(particles[ip], z_rad[:, iz], Q_rad)

    return particles


def resampling(particles):
    """
    low variance re-sampling
    """

    particles = normalize_weight(particles)

    pw = []
    for i in range(N_PARTICLE):
        pw.append(particles[i].w)

    pw = np.array(pw)

    Neff = 1.0 / (pw @ pw.T)  # Effective particle number
    # print(Neff)

    if Neff < NTH:  # resampling
        wcum = np.cumsum(pw)
        base = np.cumsum(pw * 0.0 + 1 / N_PARTICLE) - 1 / N_PARTICLE
        resampleid = base + np.random.rand(base.shape[0]) / N_PARTICLE

        inds = []
        ind = 0
        for ip in range(N_PARTICLE):
            while ((ind < wcum.shape[0] - 1) and (resampleid[ip] > wcum[ind])):
                ind += 1
            inds.append(ind)

        tparticles = particles[:]
        for i in range(len(inds)):
            particles[i].x = tparticles[inds[i]].x
            particles[i].y = tparticles[inds[i]].y
            particles[i].yaw = tparticles[inds[i]].yaw
            particles[i].lm = tparticles[inds[i]].lm[:, :]
            particles[i].lmP = tparticles[inds[i]].lmP[:, :]
            particles[i].lm_rad = tparticles[inds[i]].lm_rad[:, :]
            particles[i].lmP_rad = tparticles[inds[i]].lmP_rad[:, :]
            particles[i].lm_rad_init = tparticles[inds[i]].lm_rad_init[:, :]
            particles[i].w = 1.0 / N_PARTICLE

    return particles


def calc_input(time):

    if time <= 0.:  # wait at first
        v = 0.0
        yawrate = 0.0
    else:
        v = 1.0  # [m/s]
        yawrate = 0.1  # [rad/s]

    u = np.array([v, yawrate]).reshape(2, 1)

    return u

global STEP
STEP=0

def observation(xTrue, xd, u, RFID ,RSID):
    global STEP

    # calc true state
    xTrue = motion_model(xTrue, u)

    # add noise to range observation
    z = np.zeros((3, 0))
    z_rad = np.zeros((3,0))
    for i in range(len(RFID[:, 0])):

        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]
        d = math.sqrt(dx**2 + dy**2)
        angle = pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Qsim[0, 0]  # add noise
            anglen = angle + np.random.randn() * Qsim[1, 1]  # add noise
            zi = np.array([dn, pi_2_pi(anglen), i]).reshape(3, 1)
            z = np.hstack((z, zi))
    if STEP%1==0:
        for i in range(len(RSID[:, 0])):

            dx = RSID[i, 0] - xTrue[0, 0]
            dy = RSID[i, 1] - xTrue[1, 0]
            d = math.sqrt(dx**2 + dy**2)
            angle = pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])


            # anglen = angle + np.random.randn() * Qsim_rad[1, 1]  # add noise
            # det_output, predict_list=None,None
            #input('step')

            print(STEP,'simulation start','real',angle)
            det_output = simulate_source(angle,d)
            anglen,predict_list=get_nn_predict(det_output)
            #print(predict_list)
            print(STEP,'simulation end','predict',anglen)

            

            dn = np.random.randn() * Qsim_rad[0, 0]  # add noise
            #
            zi = np.array([dn, pi_2_pi(anglen), i]).reshape(3, 1)
            z_rad = np.hstack((z_rad, zi))

    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * Rsim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * Rsim[1, 1] + OFFSET_YAWRATE_NOISE
    ud = np.array([ud1, ud2]).reshape(2, 1)

    xd = motion_model(xd, ud)
    STEP+=1
    return xTrue, z, z_rad, xd, ud, det_output, predict_list


def motion_model(x, u):

    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT]])

    x = F @ x + B @ u

    x[2, 0] = pi_2_pi(x[2, 0])

    return x


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def main():
    global STEP
    print(__file__ + " start!!")

    time = 0.0

    # RFID positions [x, y]
    RFID = np.array([[10.0, -2.0],
                     [15.0, 10.0],
                     [15.0, 15.0],
                     [10.0, 20.0],
                     [3.0, 15.0],
                     [-5.0, 20.0],
                     [-5.0, 5.0],
                     [-10.0, 15.0]
                     ])
    RSID = np.array([[5.0,5.0]])
    N_LM = RFID.shape[0]

    # State Vector [x y yaw v]'
    xEst = np.zeros((STATE_SIZE, 1))  # SLAM estimation
    xTrue = np.zeros((STATE_SIZE, 1))  # True state
    xDR = np.zeros((STATE_SIZE, 1))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue

    particles = [Particle(N_LM) for i in range(N_PARTICLE)]

    while SIM_TIME >= time:
        time += DT
        u = calc_input(time)

        xTrue, z, z_rad, xDR, ud ,det_output, predict_list= observation(xTrue, xDR, u, RFID, RSID)

        particles = fast_slam1(particles, ud, z, z_rad)

        xEst = calc_final_state(particles)

        x_state = xEst[0: STATE_SIZE]

        # store data history
        hxEst = np.hstack((hxEst, x_state))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.plot(RFID[:, 0], RFID[:, 1], "*k")
            plt.plot(RSID[:, 0], RSID[:, 1], "ok")

            for i in range(N_PARTICLE):
                plt.plot(particles[i].x, particles[i].y, ".r")
                plt.plot(particles[i].lm[:, 0], particles[i].lm[:, 1], "xb")
                plt.plot(particles[i].lm_rad[:, 0], particles[i].lm_rad[:, 1], "+r")
                #print(particles[i].lm_rad)

            plt.plot(hxTrue[0, :], hxTrue[1, :], "-b")
            #plt.plot(hxDR[0, :], hxDR[1, :], "-k")
            plt.plot(hxEst[0, :], hxEst[1, :], "-r")
            plt.plot(xEst[0], xEst[1], "xk")
            plt.axis("equal")
            plt.grid(True)
            plt.axis([-15,20,-5,25])
            plt.pause(0.001)
            #plt.xlim(-15,15)
            #plt.ylim(-5,15)
        if record_data:
            data_dump={
            'RFID':RFID,
            'RSID':RSID,
            'particles':particles,
            'hxTrue':hxTrue,
            'hxEst':hxEst,
            'xEst':xEst,
            'det_output':det_output,
            'predict_list':predict_list
            }
            with open(os.path.join(recordpath,'STEP%.3d.pkl'%STEP),'wb') as f:
                pkl.dump(data_dump,f)
            
        #input()


if __name__ == '__main__':
    main()
