from load_bal_datasets import BALLoader
from projection import *
from utilities.math_tools import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from scipy.linalg import lu_factor, lu_solve
from scipy.linalg import lapack, solve
from scipy.sparse.linalg import spsolve_triangular
from threading import Thread
from gui import *


def undistort_point(u, K, dist_coeffs):
    # Convert to homogeneous coordinates
    u_homo = np.array([u[0], u[1], 1])
    k1 = dist_coeffs[0]
    k2 = dist_coeffs[1]
    # Normalize coordinates
    p = np.linalg.inv(K) @ u_homo

    # Apply distortion correction
    r_squared = np.sum(p[:2]**2)
    """
    The camera model is described in:
    https://grail.cs.washington.edu/projects/bal/
    """
    radial_correction = 1 + k1 * r_squared + k2 * r_squared**2

    u_corrected = np.array([
        p[0] * radial_correction,
        p[1] * radial_correction
    ])

    # Convert back to homogeneous coordinates
    u_undist_homo = np.array([u_corrected[0], u_corrected[1], 1])

    # Project back to image coordinates
    u_undist = (K @ u_undist_homo)[:2]

    return u_undist


def solveBA(graph, viewer):
    min_score_change = 1.
    last_score = np.inf
    itr = 0
    while(True):
        viewer.setVertices(graph.vertices)

        start = time.time()
        dx, score = graph.solve_once()
        end = time.time()
        itr += 1
        time_diff = end - start

        info_text = 'iter %d: solve time: %f error: %f' % (itr, time_diff, score)
        print(info_text)
        viewer.setText(info_text)
        if (last_score - score < min_score_change and itr > 5):
            print('Solved!')
            break
        graph.update(dx)
        last_score = score

if __name__ == '__main__':  

    print("Load dataset...")
    loader = BALLoader()
    filename = "data/bal/problem-49-7776-pre.txt"
    if loader.load_file(filename):
        pass
    else:
        print("Error loading file.")

    print("Undistort points...")
    for obs in loader.observations:
        cam = loader.cameras[obs.camera_id]
        obs.u_undist = undistort_point(obs.u, cam.K, cam.dist_coeffs)

    # Add noise
    #loader.points += np.random.normal(0, 0.1, loader.points.shape)
    graph = GraphSolver(use_sparse=True)

    print("Add camera vertex...")
    for i, cam in enumerate(loader.cameras):
        T = makeT(cam.R, cam.t)
        if(i == 0 or i == 1):
            # Due to the scale uncertainty, we fix the first and second frames
            graph.add_vertex(CameraVertex(T), is_constant=True)
        else:
            graph.add_vertex(CameraVertex(T))  # add vertex to graph

    print("Add point vertex...")
    for pw in loader.points:
        graph.add_vertex(PointVertex(pw))

    print("Undistort points...")
    camera_size = len(loader.cameras)
    kernel = HuberKernel(np.sqrt(5))
    for obs in loader.observations:
        cam = loader.cameras[obs.camera_id]
        graph.add_edge(ProjectEdge(
            [obs.camera_id, camera_size + obs.point_id],
            [obs.u_undist, cam.K],
            np.eye(2), kernel)) 


    app = QApplication([])
    viewer = BAViewer()
    viewer.show()

    t = Thread(target=solveBA, args=[graph, viewer])
    t.start()

    app.exec_()


