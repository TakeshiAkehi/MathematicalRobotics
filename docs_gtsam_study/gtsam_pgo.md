# Overview
```plantuml
interface NonlinerOptimizer{
    optimize() -> Values
}

class GaussNewtonOptimizer{
    NonlinearFactorGraph graph
    Values initial_estimate
    GaussNewtonParams parameters
}

class FactorGraph{
    FACTOR factors_
    size()
    add<FACTOR or CONTAINER>()
    print()
}

class NonlinearFactorGraph{}

class Factor{}
class BetweenFactor<VALUE>{
    Key key1
    Key key2
    VALUE measured
    NoiseModel model
}
class PriorFactor<VALUE>{
    Key key
    VALUE prior
    NoiseModel model
}
class Values{
    Keys _keys
    Values _values
}
class XXXFactor<VALUE>{ }
class NoiseModel{}
package ValueLike{
    class Pose3{}
    class XXXValue{}
}
note right: non-templated

NonlinerOptimizer <|-- GaussNewtonOptimizer : one of impl
FactorGraph <|-- NonlinearFactorGraph : one of impl
GaussNewtonOptimizer <.. NonlinearFactorGraph : register graph
GaussNewtonOptimizer <.. Values : register initial node values
NonlinerOptimizer ..> Values : estimated node values

Factor <|-- BetweenFactor
Factor <|-- PriorFactor
Factor <|.. XXXFactor : lot of factors are defined in GTSAM
NoiseModel *-- BetweenFactor
NoiseModel *-- PriorFactor
FactorGraph *-- Factor : a graph has many factors(edges)
PriorFactor *-- ValueLike
BetweenFactor *-- ValueLike
XXXFactor *-- ValueLike
Values <.... ValueLike : stored
```

* Optimizer : solves factor graph optimization. Optimizer is mainly related these 3 elements.
    * Nodes : initial guess / final guess
    * Edges : observation model
    * Params : optimizer option
* Nodes : the sets of unknown states to be optimized.
    * A node is probably a non-templated value that implements some functionality called a trait?
        * e.g. Pose3,Point3,...
    * Each node has identifier. typically a uint64 number.
    * Nodes are stored "Values" class which works like key-value store.
* Edges : the observation model of optimization problem.
    * A edge including these elements.
        * Two node IDs
        * The Data that discribes the difference between node2 and node1
        * The noise model that describes the precision of edge
    * There are a lot of defined types of edges in GTSAM.
        * BetweenFactorPose3 can define odometry edge, loop closure edge,..etc
        * PriorFactorPose3 can define prior node

