import numpy as np
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from utilities.math_tools import *
from utilities.plot_tools import *
from utilities import plot_tools
from graph_optimization.graph_solver import *


import gtsam
import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt


# class Pose3dEdge(BaseEdge):
#     def __init__(self, link, z, omega=np.eye(6), kernel=None):
#         super().__init__(link, z, omega, kernel)

#     def residual(self, vertices):
#         """
#         The proof of Jocabian of SE3 is given in a graph_optimization.md (18)(19)
#         """
#         Tzx = np.linalg.inv(self.z) @ vertices[self.link[0]].x
#         return logSE3(Tzx), [np.eye(6)]


# class Pose3dbetweenEdge(BaseEdge):
#     def __init__(self, link, z, omega=np.eye(6), kernel=None, color='black'):
#         super().__init__(link, z, omega, kernel)
#         self.color = color

#     def residual(self, vertices):
#         """
#         The proof of Jocabian of SE3 is given in a graph_optimization.md (18)(19)
#         """
#         Ti = vertices[self.link[0]].x
#         Tj = vertices[self.link[1]].x
#         Tij = np.linalg.inv(Ti) @ Tj

#         r = logSE3(np.linalg.inv(self.z) @ Tij)

#         Tji = np.linalg.inv(Tij)
#         Rji, tji = makeRt(Tji)
#         J = np.zeros([6, 6])
#         J[0:3, 0:3] = -Rji
#         J[0:3, 3:6] = -skew(tji) @ Rji
#         J[3:6, 3:6] = -Rji
#         return r, [J, np.eye(6)]


# class Pose3Vertex(BaseVertex):
#     def __init__(self, x):
#         super().__init__(x, 6)

#     def update(self, dx):
#         self.x = self.x @ expSE3(dx)


def draw(figname, nodes:gtsam.Values,marginals):
    size = nodes.size()
    for i in range(size):
        pose3 = nodes.atPose3(i)
        P = marginals.marginalCovariance(i)
        gtsam_plot.plot_pose3(figname,pose3,P=P)
    fig = plt.figure(figname)
    axes = fig.gca()
    set_axes_equal(figname)


PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas([0.01]*6)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas([0.02]*6)
LC_NOISE = gtsam.noiseModel.Diagonal.Sigmas([0.001]*6)

if __name__ == '__main__':
    # graph = GraphSolver()
    graph = gtsam.NonlinearFactorGraph()

    n = 12
    cur_pose = expSE3(np.array([0, 0, 0, 0, 0, 0]))
    odom = expSE3(np.array([0.2, 0, 0.00, 0.05, 0, 0.5]))
    initial_pose = cur_pose.copy()
    initial_estimate = gtsam.Values()

    # Nodes
    for i in range(n):
        # graph.add_vertex(Pose3Vertex(cur_pose))  # add vertex to graph
        cur_pose = cur_pose @ odom
        initial_estimate.insert(i,gtsam.Pose3(mat=cur_pose))

    # Prior Edge
    # graph.add_edge(Pose3dEdge([0], expSE3(np.array([0, 0, 0, 0, 0, 0]))))  # add prior pose to graph
    graph.add(
        gtsam.PriorFactorPose3(0,gtsam.Pose3(mat=initial_pose),PRIOR_NOISE)
    )

    # Odometry edge
    for i in range(n-1):
        j = (i + 1)
        # graph.add_edge(Pose3dbetweenEdge([i, j], odom))  # add edge(i, j) to graph
        graph.add(
                gtsam.BetweenFactorPose3(
                    i,i+1,gtsam.Pose3(mat=odom),ODOMETRY_NOISE
                )
            )

    # calculate covariances before loop closing
    marginals_init = gtsam.Marginals(graph, initial_estimate)

    # Loop closure edge
    # graph.add_edge(Pose3dbetweenEdge([n-1, 0], odom, color='red'))
    graph.add(
        gtsam.BetweenFactorPose3(
            n-1,0,gtsam.Pose3(),LC_NOISE
        )
    )

    graph.print()
    parameters = gtsam.GaussNewtonParams()
    optimizer = gtsam.GaussNewtonOptimizer(graph, initial_estimate, parameters)
    result = optimizer.optimize()
    marginals_post = gtsam.Marginals(graph, result)

    draw('before loop-closing', initial_estimate,marginals_init)
    draw('after loop-closing', result,marginals_post)
    # gtsam_plot.plot_trajectory(plt.figure(),values=result,marginals=marginals_post)
    plt.show()
