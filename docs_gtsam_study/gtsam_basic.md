

# Pose3 
https://gtsam.org/doxygen/4.0.0/a02711.html

## Instantiation
```
# Identity Transform
>>> gtsam.Pose3()
R: [
        1, 0, 0;
        0, 1, 0;
        0, 0, 1
]
t: 0 0 0


# From transform matrix 4x4
#  mat : numpy.ndarray[4x4]
>>> gtsam.Pose3(mat=np.eye(4))
R: [
        1, 0, 0;
        0, 1, 0;
        0, 0, 1
]
t: 0 0 0

# From vector space
#  v : [rx,ry,rz,tx,ty,tz]
#  r : rotation vector
#  t : transition vector
>>> gtsam.Pose3.Expmap(v=[0.0775253, 0.3848516, 0.4864792,1,2,3])
R: [
        0.813798, -0.44097, 0.378522;
        0.469846, 0.882564, 0.0180283;
        -0.34202, 0.163176, 0.925417
]
t: 1.05324 2.14022 2.88059

# To vector space
>>> pose3 = gtsam.Pose3.Expmap(v=[0.0775253, 0.3848516, 0.4864792,1,2,3])
>>> gtsam.Pose3.Logmap(pose3)
array([0.0775253, 0.3848516, 0.4864792, 1.       , 2.       , 3.       ])
```

rotation vector : 3-element vector whose direction represents the axis of rotation and whose magnitude represents the angle of rotation in radians. 

Supporting data :
```
Euler angles of multiple axis rotations (degrees)
ZYX : x=10, y=20, z=30

Rotation matrix
[  0.8137977, -0.4409696,  0.3785223;
   0.4698463,  0.8825641,  0.0180283;
  -0.3420202,  0.1631759,  0.9254166 ]

Axis with angle magnitude (radians) [x, y, z]
[ 0.0775253, 0.3848516, 0.4864792 ]
```
https://www.andre-gaschler.com/rotationconverter/


## Transformation

```
>>> pose3 = gtsam.Pose3.Expmap(v=[0.0775253, 0.3848516, 0.4864792,1,2,3])
>>> point3 = gtsam.Point3(np.array([1.,0.,0.]))

# To numpy matrix
>>> pose3.matrix()
array([[ 0.81379768, -0.44096958,  0.37852233,  1.05323712],
       [ 0.46984628,  0.88256413,  0.01802833,  2.14022145],
       [-0.34202018,  0.1631759 ,  0.92541657,  2.88058755],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

>>> pose3.rotation().matrix()
array([[ 0.81379768, -0.44096958,  0.37852233],
       [ 0.46984628,  0.88256413,  0.01802833],
       [-0.34202018,  0.1631759 ,  0.92541657]])

>>> pose3.translation()
array([1.05323712, 2.14022145, 2.88058755])
```

```
# Apply transformation to Point3
>>> pose3.rotation().matrix() @ point3 + pose3.translation()
array([1.8670348 , 2.61006773, 2.53856737])

## Expression above equals to transformFrom()
>>> pose3.transformFrom(point3)
array([1.8670348 , 2.61006773, 2.53856737])

## operator *(Point3) is not implemented in python
>>> pose3*point3
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
TypeError: unsupported operand type(s) for *: 'gtsam.gtsam.Pose3' and 'float'
```

```
# Apply inverse transformation to Point3
>>> pose3.inverse()
R: [
        0.813798, 0.469846, -0.34202;
        -0.44097, 0.882564, 0.163176;
        0.378522, 0.0180283, 0.925417
]
t: -0.877478  -1.89448    -3.103

>>> pose3.inverse().transformFrom(point3)
array([-0.06368027, -2.3354492 , -2.7244795 ])

>>> pose3.transformTo(point3)
array([-0.06368027, -2.3354492 , -2.7244795 ])
```

## Composition

Supporting data :
```
>>> pose3_1 = gtsam.Pose3.Expmap(v=[0.0775253, 0.3848516, 0.4864792,1,2,3])
>>> pose3_2 = gtsam.Pose3.Expmap(v=[0.1493559, 1.0663219, 0.6425684,4,5,6])
>>> point3 = gtsam.Point3(np.array([1.,0.,0.]))
```


```
pose1

Euler angles of multiple axis rotations (degrees)
ZYX : x=10, y=20, z=30

Rotation matrix
[  0.8137977, -0.4409696,  0.3785223;
   0.4698463,  0.8825641,  0.0180283;
  -0.3420202,  0.1631759,  0.9254166 ]

Axis with angle magnitude (radians) [x, y, z]
[ 0.0775253, 0.3848516, 0.4864792 ]


pose2

Euler angles of multiple axis rotations (degrees)
ZYX : x=40, y=50, z=60

Rotation matrix
[  0.3213938, -0.4172120,  0.8500825;
   0.5566704,  0.8094565,  0.1868108;
  -0.7660444,  0.4131759,  0.4924039 ]

Axis with angle magnitude (radians) [x, y, z]
[ 0.1493559, 1.0663219, 0.6425684 ]
```
https://www.andre-gaschler.com/rotationconverter/


```
# Composition of transformations (pose3_2 after pose3_1)
## Composition of transform matrix
>>> Tc = pose3_2.matrix() @ pose3_1.matrix()
>>> Tc
array([[-0.22522133, -0.37122826,  0.90081348,  6.54517023],
       [ 0.76944414,  0.49940559,  0.3981831 ,  8.98605739],
       [-0.59768811,  0.78280498,  0.17316261,  5.47070262],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

## Composition of Pose3
>>> pose3_2.transformPoseFrom(pose3_1)
R: [
        -0.225221, -0.371228, 0.900813;
        0.769444, 0.499406, 0.398183;
        -0.597688, 0.782805, 0.173163
]
t: 6.54517 8.98606  5.4707
### operator *() is available
>>> pose3_2*pose3_1
R: [
        -0.225221, -0.371228, 0.900813;
        0.769444, 0.499406, 0.398183;
        -0.597688, 0.782805, 0.173163
]
t: 6.54517 8.98606  5.4707
```