import numpy as np
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from utilities.math_tools import *
import yaml
from reprojection import *
from graph_optimization.graph_solver import *
from utilities.robust_kernel import *
from graph_optimization.plot_pose import *


class camposeNode:
    def __init__(self, x, id=0):
        self.x = x
        self.size = x.size
        self.id = id
    def update(self, dx):
        self.x = pose_plus(self.x, dx)


class featureNode:
    def __init__(self, x, id=0):
        self.x = x
        self.size = x.size
        self.id = id
    def update(self, dx):
        self.x = self.x + dx

class reporjEdge:
    def __init__(self, i, j, z, omega = None, kernel=None):
        self.i = i
        self.j = j
        self.z = z
        self.type = 'two'
        self.omega = omega
        self.kernel = kernel
        if(self.omega is None):
            self.omega = np.eye(4)
    def residual(self, nodes):
        x = nodes[self.i].x
        p = nodes[self.j].x
        xc1c2, u1, u2, xbc, K = self.z
        rl, Jl1, Jl2 = reporj(x, p, u1, K, xbc, True)
        rr, Jr1, Jr2 = reporj(x, p, u2, K, pose_plus(xbc,xc1c2), True)
        r = np.hstack([rl, rr])
        J1 = np.vstack([Jl1, Jr1])
        J2 = np.vstack([Jl2, Jr2])
        return r, J1, J2
    
def draw3d(figname, frames, points, x_bc):
    pts = []
    for i in points:
        x = points[i]['p3d']
        #x = transform(x_bc, x)
        pts.append(x)
    pts = np.array(pts)
    for frame in frames:
        x_wb = frame['pose']
        x_wc = pose_plus(x_wb, x_bc)
        T = tom(x_wb)
        plot_pose3(figname, T, 0.05)
    fig = plt.figure(figname)
    axes = fig.gca()
    axes.scatter(pts[:,0],pts[:,1],pts[:,2])
    set_axes_equal(figname)


def calc_camera_pose(frame, points, x_wc, K, x_c1c2, x_bc):
    graph = graphSolver()
    idx = graph.addNode(camposeNode(x_wc)) 
    for p in frame['points']:
        if not p in points:
            continue
        u0 = frame['points'][p][0:2]
        u1 = u0.copy()
        u1[0] -= frame['points'][p][2]
        p3d = points[p]['p3d']
        idx_p = graph.addNode(featureNode(p3d),True) 
        graph.addEdge(reporjEdge(idx, idx_p, [x_c1c2, u0, u1, x_bc, K],kernel=CauchyKernel(0.5)))
    graph.solve(False)
    return graph.nodes[idx].x

def initmap(frames, K, x_c1c2, x_bc):
    baseline = 0.075
    focal = K[0,0]
    Kinv = np.linalg.inv(K)
    points = {}
    for i, frame in enumerate(frames):
        print("initial frame %d..."%i)
        if(i != 0):
            x_wc = calc_camera_pose(frame, points, frames[i-1]['pose'],K, x_c1c2, x_bc)
            frames[i]['pose'] = x_wc
        for j in list(frame['points']):
            if j in points:
                points[j]['view'].append(i)
                continue
            u,v,disp = frame['points'][j]
            if(disp < 20):
                frame['points'].pop(j)
                continue
            p3d_c = Kinv.dot(np.array([u,v,1.]))
            depth = (baseline * focal) / (disp)
            p3d_c *= depth
            p3d_b = transform(x_bc, p3d_c)
            p3d_w = transform(frames[i]['pose'], p3d_b)
            points.update({j: {'view':[i],'p3d':p3d_w}})
            #u1 = reporj(frames[i]['pose'], p3d_w, np.array([u,v]), K, x_bc, calcJ = False)
            #u2 = reporj(frames[i]['pose'], p3d_w, np.array([u-disp,v]), K, pose_plus(x_bc, x_c1c2), calcJ = False)
            #print(u1,u2)
    return points

def remove_outlier(frames, points, K, xc2c1):
    for i, frame in enumerate(frames):
        x = frame['pose']
        for j in list(frame['points']):
            if j in points:
                pw = points[j]['p3d']
                u0 = frame['points'][j][0:2]
                u1 = u0.copy()
                u1[0] -= frame['points'][j][2]
                u0_reproj = reporj(x, pw, np.zeros(2), K)
                u1_reproj = reporj(pose_plus(xc2c1,x), pw, np.zeros(2), K)
                d0 = np.linalg.norm(u0_reproj - u0)
                d1 = np.linalg.norm(u1_reproj - u1)
                d = d0 + d1
                if(d > 2):
                    del frame['points'][j]
                    idx = points[j]['view'].index(i)
                    points[j]['view'].pop(idx)
                    if(len(points[j]['view']) == 0):
                        del points[j]
                        break





def draw_frame(frames, points, K, x_c1c2, x_bc):
    for frame in frames:
        x_wb = frame['pose']
        u0s = []
        u1s = []
        for j in frame['points']:
            if j in points:
                pw = points[j]['p3d']
                u0 = frame['points'][j][0:2]
                u1 = reporj(x_wb, pw, np.zeros(2), K, x_bc)
                u0s.append(u0)
                u1s.append(u1)
        u0s = np.array(u0s)
        u1s = np.array(u1s)
        plt.xlim(0,640)
        plt.ylim(0,400)
        plt.gca().invert_yaxis()
        plt.scatter(u0s[:,0],u0s[:,1])
        plt.scatter(u1s[:,0],u1s[:,1])
        plt.grid()
        plt.show()

def readframes(n,folder):
    frames = []
    for idx in range(0,n):
        fn = folder+'/F%04d.yaml'%idx
        print('read %s...'%fn)
        with open(fn) as file:
            node = yaml.safe_load(file)
            pts = np.array(node['points']['data']).reshape(node['points']['num'],-1)
            pts = dict(zip(pts[:,0].astype(np.int), pts[:,1:].astype(np.float)))
            imus = np.array(node['imu']['data']).reshape(node['imu']['num'],-1)
            frames.append({'stamp':node['stamp'],'pose':np.zeros(6),'points': pts,'imu':imus})
    return frames

def solve(frames, points, K, x_c1c2, x_bc):
    graph = graphSolver()
    frames_idx = {}
    points_idx = {}
    for i, frame in enumerate(frames):
        x_wc = frame['pose']
        idx = graph.addNode(camposeNode(x_wc, i),i==0) # add node to graph
        frames_idx.update({i: idx})
    for j in points:
        idx = graph.addNode(featureNode(points[j]['p3d'], j)) # add feature to graph
        points_idx.update({j: idx})
        for i in points[j]['view']:
            f_idx = frames_idx[i]
            u0 = frames[i]['points'][j][0:2]
            u1 = u0.copy()
            u1[0] -= frames[i]['points'][j][2]
            graph.addEdge(reporjEdge(f_idx, idx, [x_c1c2, u0, u1, x_bc, K],kernel=CauchyKernel(0.1)))      
    graph.report()
    graph.solve()
    graph.report()
    for n in graph.nodes:
        if( type(n).__name__ == 'featureNode'):
            points[n.id]['p3d'] = n.x
        if( type(n).__name__ == 'camposeNode'):
            frames[n.id]['pose'] = n.x

if __name__ == '__main__':
    fx = 403.5362854003906
    fy = 403.4488830566406
    cx = 323.534423828125
    cy = 203.87405395507812
    x_c1c2 = np.array([0,0,0,0.075,0,0])
    x_bc = np.array([0,0,np.pi/2,0.0,0,0])
    K = np.array([[fx,0, cx],[0, fy,cy],[0,0,1.]])

    frames = readframes(2, 'data/slam')
    points = initmap(frames, K, x_c1c2, x_bc)
    solve(frames, points, K, x_c1c2, x_bc)
    remove_outlier(frames, points, K, x_c1c2)
    solve(frames, points, K, x_c1c2, x_bc)

    draw3d('view',frames, points, x_bc)
    #draw_frame(frames, points, K, x_c1c2, x_bc)
    plt.show()