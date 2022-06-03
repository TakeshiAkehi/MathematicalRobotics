
from typing import Optional, Sequence

import gtsam
import matplotlib.pyplot as plt
import numpy as np
from gtsam.utils.plot import plot_pose3
from mpl_toolkits.mplot3d import Axes3D
from gtsam.symbol_shorthand import B, V, X
import sys,os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
#from utilities.math_tools import *
from graph_optimization.plot_pose import *

IMU_FIG = 1
POSES_FIG = 2
GRAVITY = 9.80


#pose_data = np.load('/home/liu/bag/pose0.npy') 
#imu_data = np.load('/home/liu/bag/imu0.npy')
#pose_file = '/home/liu/bag/warehouse/2022-06-01-19-06-46_pose.npy'
pose_file = '/home/liu/bag/warehouse/b2_pose.npy'
pose_data = np.load(pose_file) 
pose_data = pose_data[::20]
imu_data = np.load('/home/liu/bag/warehouse/b2_imu.npy')
pose_truth_data = np.load('/home/liu/bag/warehouse/b2_pose.npy')


imu_params = gtsam.PreintegrationParams.MakeSharedU(9.81)
imu_params.setAccelerometerCovariance(np.eye(3) * np.power( 0.01, 2))
imu_params.setIntegrationCovariance(np.eye(3) * np.power( 0, 2))
imu_params.setGyroscopeCovariance(np.eye(3) * np.power(  0.00175, 2))
imu_params.setOmegaCoriolis(np.zeros(3))
imuIntegratorOpt = gtsam.PreintegratedImuMeasurements(imu_params, gtsam.imuBias.ConstantBias())

priorPoseNoise  = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2]) ) # rad,rad,rad,m, m, m
odom = 1e-5
odomR = 1e-2
OdometryNoise  = gtsam.noiseModel.Diagonal.Sigmas(np.array([odomR, odomR, odomR, odom, odom, odom]) ) 
priorVelNoise = gtsam.noiseModel.Isotropic.Sigma(3, 100) 
priorBiasNoise = gtsam.noiseModel.Isotropic.Sigma(6, 1e-4) 
correctionNoise = gtsam.noiseModel.Diagonal.Sigmas(np.array([ 0.05, 0.05, 0.05, 1, 1, 1]))

optParameters = gtsam.ISAM2Params()
optParameters.setRelinearizeThreshold(0.1)
optimizer = gtsam.ISAM2(optParameters)
graphFactors = gtsam.NonlinearFactorGraph()
graphValues = gtsam.Values()

imuAccBiasN = 6.4356659353532566e-10
imuGyrBiasN = 3.5640318696367613e-10
noiseModelBetweenBias = np.array([imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN])

initb = gtsam.imuBias.ConstantBias(np.array([0.007,-0.038,-0.01]),np.array([0,0,0]))
n= pose_data.shape[0]
n= 200
#Add nodes to graph
imu_group = []
begin_idx = 0
for i in range(n):
    p0 = pose_data[i]
    curPose = gtsam.Pose3(gtsam.Rot3.Quaternion(*p0[4:8]), gtsam.Point3(*p0[1:4]))
    if(i == n - 1):
        graphValues.insert(X(i), curPose)
        graphValues.insert(V(i), graphValues.atPoint3(V(i-1)))
        graphValues.insert(B(i), initb)
        break
    else:
        p1 = pose_data[i+1]
        dt = p1[0] - p0[0]
        vel = (p1[1:4] - p0[1:4])/dt
        vel[2] = 0
        graphValues.insert(X(i), curPose)
        graphValues.insert(V(i), vel)
        graphValues.insert(B(i), initb)
    tmp = []
    for imu in imu_data[begin_idx:]:
        begin_idx += 1
        if(imu[0]< p0[0] ):
            continue
        if(imu[0] > p1[0]):
            break
        if dt <= 0:
            continue
        tmp.append(imu)
    imu_group.append(tmp)


#Add edges to graph
graphFactors.add(gtsam.PriorFactorPose3(X(0), graphValues.atPose3(X(0)), priorPoseNoise))
graphFactors.add(gtsam.PriorFactorPoint3(V(0), graphValues.atPoint3(V(0)), priorVelNoise))
graphFactors.add(gtsam.PriorFactorConstantBias(B(0), graphValues.atConstantBias(B(0)), priorBiasNoise))

for i in range(n-1):
    pass
    imuIntegratorOpt.resetIntegrationAndSetBias(gtsam.imuBias.ConstantBias())
    for imu in imu_group[i]:
        imuIntegratorOpt.integrateMeasurement(imu[1:4], imu[4:7], 0.005)
    imu_factor = gtsam.ImuFactor(X(i), V(i), X(i+1), V(i+1), B(i), imuIntegratorOpt)
    graphFactors.add(imu_factor)
    bias_factor = gtsam.BetweenFactorConstantBias( B(i), B(i+1), gtsam.imuBias.ConstantBias(), \
        gtsam.noiseModel.Diagonal.Sigmas( np.sqrt(imuIntegratorOpt.deltaTij())* noiseModelBetweenBias ))
    graphFactors.add(bias_factor)
    vel_factor = gtsam.BetweenFactorPoint3( V(i), V(i+1), gtsam.Point3(0,0,0), \
        gtsam.noiseModel.Isotropic.Sigma(3, 1e-4)  )
    #graphFactors.add(vel_factor)
    graphFactors.add(gtsam.PriorFactorPoint3(V(i), graphValues.atPoint3(V(i)), gtsam.noiseModel.Isotropic.Sigma(3, 1e-5) ))

    odometry = graphValues.atPose3(X(i)).inverse()*graphValues.atPose3(X(i+1))
    graphFactors.add(gtsam.BetweenFactorPose3(X(i), X(i+1), odometry, OdometryNoise))


optimizer.update(graphFactors, graphValues)
optimizer.update()

result = optimizer.calculateEstimate()
#for i in range(n):
    #plot_pose3("1",logSE3(result.atPose3(X(i)).matrix()),0.1)
#set_axes_equal("1")


bias_acc_opt = []
bias_gyo_opt = []
pose_opt = pose_data.copy()
vel_opt = []
for i in range(n-1):
    #pose.append(result.atPose3(X(i)).translation())
    pose_opt[i,1:4] = result.atPose3(X(i)).translation()
    bias_acc_opt.append(result.atConstantBias(B(i)).accelerometer())
    bias_gyo_opt.append(result.atConstantBias(B(i)).gyroscope())
    vel_opt.append(result.atPoint3(V(i)))

bias_acc_opt = np.array(bias_acc_opt)
bias_gyo_opt = np.array(bias_gyo_opt)
pose_opt = np.array(pose_opt)
vel_opt = np.array(vel_opt)
np.save(os.path.splitext(pose_file)[0][:-5]+'_opt_pose.npy', pose_opt)

fig = plt.figure('imu')
plt.plot(imu_data[0:n,1],color='r', label='acc x')
plt.plot(imu_data[0:n,2],color='g', label='acc y')
plt.plot(imu_data[0:n,3],color='b', label='acc z')
plt.legend()

fig = plt.figure('acc bias')
plt.plot(bias_acc_opt[0:n,0],color='r', label='bias x')
plt.plot(bias_acc_opt[0:n,1],color='g', label='bias y')
plt.plot(bias_acc_opt[0:n,2],color='b', label='bias z')
plt.legend()

fig = plt.figure('gyo bias')
plt.plot(bias_gyo_opt[0:n,0],color='r', label='bias x')
plt.plot(bias_gyo_opt[0:n,1],color='g', label='bias y')
plt.plot(bias_gyo_opt[0:n,2],color='b', label='bias z')
plt.legend()

fig = plt.figure('vel')
plt.plot(vel_opt[0:n,0],color='r', label='vel x')
plt.plot(vel_opt[0:n,1],color='g', label='vel y')
plt.plot(vel_opt[0:n,2],color='b', label='vel z')
plt.legend()

fig = plt.figure('pose')
plt.scatter(pose_data[0:n,1],pose_data[0:n,2],s=5,label = 'raw path')
plt.scatter(pose_opt[0:n,1],pose_opt[0:n,2],s=5,label = 'imu path')
plt.scatter(pose_truth_data[0:n*20,1],pose_truth_data[0:n*20,2],s=5,label = 'truth path')

plt.legend()
plt.show()