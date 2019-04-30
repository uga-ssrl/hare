"""
Ros version of Extended kalman filter (EKF) localization sample

original author: Atsushi Sakai (@Atsushi_twi)

modified by Ben Ware(@BenWare144) and Allen Spain (@Spain2394)
"""
import numpy as np
import math
import matplotlib.pyplot as plt
import random

# # Estimation parameter of EKF
Q = np.diag([0.1, 0.1, 0.1, 0.1])**2
R = np.diag([1.0, 1.0])**2
DT = 1  # Descrete time steps
SIM_TIME = 100

def get_rand(): return random.choice([1,1,1,0])

show_animation = True

## Assumptions and notes:
# Input noise is simulated with a 25% chance to drop input and not regester the velocity.
# Robot does not have prior knowledge of chance of "slipping"
# Robot does not assume it is confined to a descrete domain.

def check_input(u, uTrue, z, zTrue):
    #check u  
    if type(u)==list:
        u=np.matrix(u).T
    elif len(z.shape)==1: 
        u=np.matrix(u.tolist()).T
    elif len(z.shape)==2 and u.shape[0] == 1 and u.shape[1] == 2:
            u = u.T
    #check uTrue       
    if type(uTrue)==list:
        uTrue=np.matrix(uTrue).T
    elif len(z.shape)==1: 
        uTrue=np.matrix(uTrue.tolist()).T
    elif len(z.shape)==2 and uTrue.shape[0] == 1 and uTrue.shape[1] == 2:
            uTrue = uTrue.T
    #check z
    if type(z)==list:
        z=np.matrix(z)
    elif len(z.shape)==1: 
        z=np.matrix(z.tolist())
    elif len(z.shape)==2 and z.shape[0] == 2 and z.shape[1] == 1:
            z = z.T
    #check zTrue
    if type(zTrue)==list:
        zTrue=np.matrix(zTrue)
    elif len(zTrue.shape)==1: 
        zTrue=np.matrix(zTrue.tolist())
    elif len(zTrue.shape)==2 and zTrue.shape[0] == 2 and zTrue.shape[1] == 1:
            zTrue = zTrue.T
    #return
    return (u, uTrue, z, zTrue)


def simulate_input():
    [v_i,v_j] = random.choice([[1,0],[-1,0],[0,1]]) # omitted [0,-1] to create tendency to move upward
    u = np.matrix([v_i, v_j]).T 
    return u

def simulate_measurement(xTrue):
    #gps i,j
    i = xTrue[0, 0]
    j = xTrue[1, 0]
    z = np.matrix([i, j])
    return z

def simulate_input_noise(u):
    ui = u[0, 0]*get_rand()
    uj = u[1, 0]*get_rand()
    u = np.matrix([ui, uj]).T
    return u
def simulate_measurement_noise(z):
    # add noise to gps i-j
    i = z[0,0] + np.random.randn()//2 
    j = z[0,1] + np.random.randn()//2 
    z = np.matrix([i, j])
    return z


def motion_model(x, u):
#     u is the change in location or "velocity"
#     x = position i,j
#     velocities are independent of each other
    F = np.matrix([[1.0, 0, 0, 0],
                   [0, 1.0, 0, 0],
                   [0, 0, 0, 0],
                   [0, 0, 0, 0]])

    B = np.matrix([[1.0, 0],
                   [0, 1.0],
                   [1.0, 0],
                   [0, 1.0]])

    x = F * x + B * u
    return x
def observation_model(x):
    #  Observation Model
    H = np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
#         [1, 0],
#         [0, 1]
    ])
    z = H * x
    return z


def jacobF(x, u):
    """
    Jacobian of Motion Model

    motion model
    x_{t+1} = x_t+v_x
    y_{t+1} = y_t+v_y
    x_t{t+1} = v_x
    y_t{t+1} = v_y
    so
    dx/dv_x = 1 
    dy/dv_y = 1
    """
    v_x =u[0, 0]                
    v_y =u[1, 0]                
    jF = np.matrix([ 
         [1.0, 0.0, 1, 0],
         [0.0, 1.0, 0, 1],
         [0.0, 0.0, 1.0, 0.0],
         [0.0, 0.0, 0.0, 1.0]])
    return jF



def jacobH(x):
    # Jacobian of Observation Model
    jH = np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])
    return jH



def ekf_estimation(xEst, PEst, z, u):

    #  Predict
    xPred = motion_model(xEst, u)
    jF = jacobF(xPred, u)
    PPred = jF * PEst * jF.T + Q

    #  Update
    jH = jacobH(xPred)
    zPred = observation_model(xPred)
    y = z.T - zPred
    S = jH * PPred * jH.T + R
    K = PPred * jH.T * np.linalg.inv(S)
    xEst = xPred + K * y
    PEst = (np.eye(len(xEst)) - K * jH) * PPred

    return xEst, PEst


def plot_covariance_ellipse(xEst, PEst):
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
    R = np.matrix([[math.cos(angle), math.sin(angle)],
                   [-math.sin(angle), math.cos(angle)]])
    fx = R * np.matrix([x, y])
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")
    
def graph_hist(hz,hxTrue,hxDR,hxEst,xEst,PEst):
        plt.cla()
        plt.plot(hz[:, 0], hz[:, 1], ".g")
        plt.plot(np.array(hxTrue[0, :]).flatten(),
                 np.array(hxTrue[1, :]).flatten(), "-b")
        plt.plot(np.array(hxDR[0, :]).flatten(),
                 np.array(hxDR[1, :]).flatten(), "-k")
        plt.plot(np.array(hxEst[0, :]).flatten(),
                 np.array(hxEst[1, :]).flatten(), "-r")
        plot_covariance_ellipse(xEst, PEst)
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)
    
class kalman_filter:
    def __init__(self):
        # State Vector [x y yaw v]'
        self.xEst = np.matrix(np.zeros((4, 1)))
        self.xTrue = np.matrix(np.zeros((4, 1)))
        self.PEst = np.eye(4)
        self.xDR = np.matrix(np.zeros((4, 1)))  # Dead reckoning
        # history
        self.hxEst = self.xEst
        self.hxTrue = self.xTrue
        self.hxDR = self.xTrue
        self.hz = np.zeros((1, 2))
        self.hu = np.zeros((1, 2))
        
        
    def move_forward(self,u,uTrue,z,zTrue,generate_input_noise = False,generate_measurement_noise = False):
        #main function
        u, uTrue, z, zTrue = check_input(u, uTrue, z, zTrue)
        if generate_input_noise: u = simulate_input_noise(uTrue) # maybe add input_noise
        if generate_measurement_noise: z = simulate_measurement_noise(zTrue) # maybe add measurement_noise
        self.simulate_DR(u)
        self.estimate(u,z)
        self.xTrue = motion_model(self.xTrue, uTrue)
        self.store_history(u,z)
        return (self.xEst.item(0), self.xEst.item(1))
    
    def simulate_inputs(self):
        uTrue = simulate_input()
        xTrue = motion_model(self.xTrue, uTrue)
        zTrue = simulate_measurement(xTrue)
        return (uTrue, uTrue, zTrue, zTrue)
        
    def simulate_DR(self,u):
        # simulate DR movement (i.e. localize robot as if it had only positional data from the start)
        self.xDR = motion_model(self.xDR, u)
        
    def estimate(self,u,z):
        self.xEst, self.PEst = ekf_estimation(self.xEst, self.PEst, z, u)

    def store_history(self,u,z):
        # store data history
        self.hxEst  = np.hstack((self.hxEst, self.xEst))
        self.hxDR   = np.hstack((self.hxDR, self.xDR))
        self.hxTrue = np.hstack((self.hxTrue, self.xTrue))
        self.hu     = np.vstack((self.hu, u.T))
        self.hz     = np.vstack((self.hz, z))
    
    def graph(self):
        graph_hist(self.hz,self.hxTrue,self.hxDR,self.hxEst,self.xEst,self.PEst)
        
#example usage
# def main():
#     kf = kalman_filter()
#     for _ in range(100):
#         u, uTrue, z, zTrue = kf.simulate_inputs()
#         print(kf.move_forward(u,uTrue,z,zTrue))
#         kf.graph()
    
# if __name__ == '__main__':
#     main()
