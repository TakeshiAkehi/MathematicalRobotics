
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from graph_optimization.graph_solver import *
from utilities.math_tools import *
from preintegration import *
import numpy as np
import matplotlib.pyplot as plt
from graph_optimization.plot_pose import *
import quaternion


class naviEdge:
    def __init__(self, i, z):
        self.i = i
        self.z = z
        self.type = 'one'
        self.omega = np.eye(9)
    def residual(self, nodes):
        """
        The proof of Jocabian of SE3 is given in a graph_optimization.md (20)(21)
        """
        state = nodes[self.i].state
        r, j, _ = state.local(self.z, True)
        r = r.vec()
        return r, j


class imuEdge:
    def __init__(self, i, j, k, z):
        self.i = i
        self.j = j
        self.k = k
        self.z = z
        self.type = 'three'
        self.omega = np.eye(9)
    def residual(self, nodes):
        pim = self.z
        statei = nodes[self.i].state
        statej = nodes[self.j].state
        bias = nodes[self.k].bias
        statejstar, J_statejstar_statei, J_statejstar_bias = pim.predict(statei, bias, True)
        r, J_local_statej, J_local_statejstar = statej.local(statejstar, True)
        r = r.vec()
        J_statei = J_local_statejstar.dot(J_statejstar_statei)
        J_statej = J_local_statej
        J_biasi = J_local_statejstar.dot(J_statejstar_bias)
        return r, J_statei, J_statej, J_biasi


class naviNode:
    def __init__(self, state):
        self.state = state
        self.size = 9
    def update(self, dx):
        d_state = navState(expSO3(dx[0:3]), dx[3:6], dx[6:9])
        self.state = self.state.retract(d_state)

class biasNode:
    def __init__(self, bias):
        self.bias = bias
        self.size = 6
    def update(self, dx):
        self.bias = self.bias + dx

def to2d(x):
    R = expSO3(x[0:3])
    theta = np.arctan2( R[1,0], R[0,0])
    x2d = np.zeros(3)
    x2d[0:2] = x[3:5]
    x2d[2] = theta
    return x2d

def draw(figname, gs, color):
    fig = plt.figure(figname)
    axes = fig.gca()

    pose_trj = []
    for n in gs.nodes:
        if(not isinstance(n, naviNode)):
            continue
        pose_trj.append(n.state.p)
    pose_trj = np.array(pose_trj)
    axes.scatter(pose_trj[:,0],pose_trj[:,1], c=color, s=10, label='pose')
    imu_trj = []
    for e in gs.edges:
        if(not isinstance(e, imuEdge)):
            continue
        imuIntegrator = imuIntegration(9.80)
        statei = gs.nodes[e.i].state
        biasi = gs.nodes[e.k].bias
        for acc, gyo, dt in zip(e.z.acc_buf, e.z.gyo_buf, e.z.dt_buf):
            imuIntegrator.update(acc, gyo, dt)
            state_new = imuIntegrator.predict(statei,biasi)
            imu_trj.append(state_new.p)
    imu_trj = np.array(imu_trj)
    axes.scatter(imu_trj[:,0],imu_trj[:,1], c=color, s=2,label='imu predict')

if __name__ == '__main__':
    imuIntegrator = imuIntegration(9.80)
    pose_file = '/home/liu/bag/warehouse/b2_mapping_pose.npy'
    #pose_file = '/home/liu/bag/warehouse/b2_pose.npy'
    pose_data = np.load(pose_file) 
    imu_data = np.load('/home/liu/bag/warehouse/b2_imu.npy')
    gs = graphSolver()
    n= pose_data.shape[0]
    #n= 100
    imu_group = []
    begin_idx = 0
    last_state_idx = 0
    last_state_stamp = 0
    for i in range(n):
        p = pose_data[i]
        if(i == 0):
            state = navState(quaternion.as_rotation_matrix(np.quaternion(*p[4:8])),p[1:4],np.array([0,0,0]))
            last_state_idx = gs.addNode(naviNode(state))
            gs.addEdge(naviEdge(last_state_idx, state)) 
            last_state_stamp = p[0]
        else:
            #p0 = pose_data[i-1]
            last_p = gs.nodes[last_state_idx].state.p
            dt = p[0] - last_state_stamp
            dist = np.linalg.norm(p[1:4] - last_p)
            if(dist < 0.1):
                continue
            vel = (p[1:4] - last_p)/dt
            state = navState(quaternion.as_rotation_matrix(np.quaternion(*p[4:8])),p[1:4],vel)
            cur_state_idx = gs.addNode(naviNode(state)) # add node to graph
            bias_idx = gs.addNode(biasNode(np.array([0,0,0,0,0,0]))) # add node to graph
            gs.addEdge(naviEdge(cur_state_idx, state)) 
            imuij = []
            for imu in imu_data[begin_idx:]:
                begin_idx += 1
                if(imu[0]< last_state_stamp ):
                    continue
                if(imu[0] > p[0]):
                    break
                if dt <= 0:
                    continue
                imuij.append(imu)
            imuIntegrator = imuIntegration(9.81)
            for imu in imuij:
                imuIntegrator.update(imu[1:4], imu[4:7], 0.01)
            gs.addEdge(imuEdge(last_state_idx, cur_state_idx, bias_idx, imuIntegrator))
            last_state_idx = cur_state_idx
            last_state_stamp = p[0]
    draw("1",gs,'red')
    print("score:%f"%gs.getScore())
    gs.solve()
    draw("1",gs,'green')
    print("score:%f"%gs.getScore())
    plt.legend()
    plt.show()


"""
if __name__ == '__main__':
    imuIntegrator = imuIntegration(9.81)
    last_stamp = -1
    statei = navState(np.array([0,0,0]),np.array([0,0,0]),np.array([0,0,0]))
    statej = navState(np.array([-7.15181825e-06, -2.53153786e-05,  0.5]),\
                      np.array([ 2,  0.2, 0]),
                      np.array([ 1.56498744,  0.00278504, -0.08635731]))
    trj0 = []
    for i in imu:
        cur_stamp = i[0]
        dt = 0
        if(last_stamp < 0):
            dt = 0.01
        else:
            dt = cur_stamp - last_stamp
        if dt <= 0:
            continue
        imuIntegrator.update(i[1:4], i[4:7], dt)
        if(imuIntegrator.d_tij > 8):
            break
        
    gs = graphSolver()
    gs.addNode(naviNode(statei)) # add node to graph
    gs.addNode(naviNode(statej)) # add node to graph
    gs.addNode(biasNode(np.array([0,0,0,0,0,0]))) # add node to graph
    gs.addEdge(naviEdge(0, statei)) 
    gs.addEdge(naviEdge(1, statej)) 
    gs.addEdge(imuEdge(0, 1, 2, imuIntegrator)) 

    draw(1,gs)
    gs.solve()
    draw(2,gs)
    plt.legend()
    plt.show()
"""