import numpy as np
import scipy
import matplotlib.pyplot as plt
import sys
import os

import scipy.optimize
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from utilities.math_tools import *
from guass_newton import *
from mpl_toolkits.mplot3d import axes3d, Axes3D
from utilities.pcd_io import load_pcd

class Plot3D:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        # elev = -107.
        # azim = 0
        # self.ax.view_init(elev, azim)

    def update(self, a, b):
        plt.cla()
        self.ax.scatter3D(a[:, 0], a[:, 1], a[:, 2], c='r')
        self.ax.scatter3D(b[:, 0], b[:, 1], b[:, 2], c='b')
        plt.pause(0.2)

    def show(self):
        plt.show()

class Original:
    @classmethod
    def residual(cls,T, param):
        """
        The residual vector of 3d point matching is given by guass_newton_method.md (7)
        The proof of Jocabian of 3d point matching is given in a guass_newton_method.md (12)
        """
        a, b = param
        R, t = makeRt(T)
        r = R.dot(a) + t - b
        I = np.eye(3)
        j = np.zeros([4, 6])
        j[0:3, 0:3] = I
        j[0:3, 3:6] = skew(-a)
        j = (T @ j)[0:3]
        return r.flatten(), j



def residuals(x,*args,**kwargs):
    params = kwargs["params"]
    T = expSE3(x)
    r = np.full((len(params),3),fill_value=np.nan)
    for i,param in enumerate(params):
        a,b = param
        R, t = makeRt(T)
        r_i = R.dot(a) + t - b
        r[i,:] = r_i
    ret = r.flatten()
    return ret

def jacobian(x,*args,**kwargs):
    params = kwargs["params"]
    T = expSE3(x)
    J = np.full((len(params)*3,6),fill_value=np.nan)
    for i,param in enumerate(params):
        a,_ = param
        J_i = np.zeros((4,6))
        J_i[0:3,0:3] = np.eye(3)
        J_i[0:3,3:6] = skew(-a)
        j = (T@J_i)[0:3,:]
        J[i*3:(i*3)+3,:] = j.copy()
    return J

if __name__ == "__main__":
    plt3d = Plot3D()

    T_truth = expSE3(np.array([1000, -0.1, 0.1, 2.1, 2.2, -1.3]))
    # T_truth = expSE3(np.array([10, -0.1, 0.1, 2.1, 2.2, -1.3]))
    a = load_pcd("data/bunny.pcd")
    b = transform3d(T_truth, a.T).T
    elements = a.shape[0]
    b += np.random.normal(0, 0.001, (elements, 3))
    x0 = np.array([0,0,0,0,0,0],dtype=np.float64)
    params = []
    for i in range(a.shape[0]):
        params.append([a[i], b[i]])
    kwargs = {"params": params}

    # print(f"size of x : {x0.shape}")
    # print(f"size of pts : {a.shape[0]}")
    # print(f"size of res : {residuals(x0,**kwargs).shape}")
    # print(f"size of jac : {jacobian(x0,**kwargs).shape}")
    
    ### very slow
    res = scipy.optimize.least_squares(fun=residuals,x0=x0,kwargs=kwargs,verbose=2,max_nfev=6000)

    ### (needed plus operator??)
    # res = scipy.optimize.least_squares(fun=residuals,jac=jacobian,x0=x0,kwargs=kwargs,verbose=2)


    T_opt = expSE3(res.x)
    a_opt = transform3d(T_opt,a.T).T
    plt3d.update(a_opt, b)
    plt3d.show()
    
    a = 1

