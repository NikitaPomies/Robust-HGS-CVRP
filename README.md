
# Robust-HGS-CVRP: An adaptation of HGS-CVRP to solve a robust routing problem

This code is an adaptation of Thibault Vidal's work to solve a modified version of a CVRP for a school project.

If using MTZ constraints, our robust cvrp can be formulated as follow : 

```math
\begin{align*}
 {min}_{\,x}~& \; {max}_{\, \delta^1,\delta^2} \sum_{(i,j)\in A} t_{ij} \, x_{ij} + (\delta_{ij}^1(\hat{t}_i + \hat{t}_j) + \delta_{ij}^2 \hat{t_i}\hat{t_j}) \, x_{ij} & \\
 {s.t} &\sum_{i \in [n]} x_{ij} = 1& \forall j \in [2,n]\\
  &\sum_{i \in [n]} x_{ji} = 1& \forall j \in [2,n]\\
 &w_i - w_j \leq C \, (1 - x_{ij}) - d_j &\forall (i,j) \in A\\
& d_i\leq w_i \leq C  & \forall i \in [n]\\ 
& \sum_{(i,j)\in A} \delta_{ij}^1 \leq T \\
& \sum_{(i,j)\in A} \delta_{ij}^2 \leq T^2 \\
& x_{ij} \in \{0,1\}& \forall (i,j) \in A\\
& \delta_{ij}^1 \in [0,1], \delta_{ij}^2 \in [0,2] & \forall (i,j) \in A 
\end{align*}
```
Exact methods( Dualisation, Cutting Planes, etc.) were tested to solve this problem and can be found at https://github.com/NikitaPomies/RobustCVRP.

## Modification
The main made changes are in the LocalSearch and Individual class. Each move must now take into account the robust cost instead of the static one. In the original code, computing the incremental ccost of move is in O(1), which is not the case now for our robust problem. We designed an algorithm to compute the incremental robust cost that still had correct time performances overall.

The fitness was also modified to take into account the robust cost



## References



[1] Vidal, T., Crainic, T. G., Gendreau, M., Lahrichi, N., Rei, W. (2012). 
A hybrid genetic algorithm for multidepot and periodic vehicle routing problems. Operations Research, 60(3), 611-624. 
https://doi.org/10.1287/opre.1120.1048 (Available [HERE](https://w1.cirrelt.ca/~vidalt/papers/HGS-CIRRELT-2011.pdf) in technical report form).

[2] Vidal, T. (2022). Hybrid genetic search for the CVRP: Open-source implementation and SWAP* neighborhood. Computers & Operations Research, 140, 105643.
https://doi.org/10.1016/j.cor.2021.105643 (Available [HERE](https://arxiv.org/abs/2012.10384) in technical report form).



## Compiling the executable 

You need [`CMake`](https://cmake.org) to compile.

Build with:
```console
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles"
make bin
```
This will generate the executable file `hgs` in the `build` directory.

## Running the algorithm

The following options are supported:
```
Call with: ./hgs instancePath solPath [-it nbIter] [-t myCPUtime] [-bks bksPath] [-seed mySeed] [-veh nbVehicles] [-log verbose]
[-it <int>] sets a maximum number of iterations without improvement. Defaults to 20,000                                     
[-t <double>] sets a time limit in seconds. If this parameter is set, the code will be run iteratively until the time limit           
[-seed <int>] sets a fixed seed. Defaults to 0                                                                                    
[-veh <int>] sets a prescribed fleet size. Otherwise a reasonable UB on the fleet size is calculated                      
[-round <bool>] rounding the distance to the nearest integer or not. It can be 0 (not rounding) or 1 (rounding). Defaults to 1. 
[-log <bool>] sets the verbose level of the algorithm log. It can be 0 or 1. Defaults to 1.                                       

Additional Arguments:
[-nbIterTraces <int>] Number of iterations between traces display during HGS execution. Defaults to 500
[-nbGranular <int>] Granular search parameter, limits the number of moves in the RI local search. Defaults to 20               
[-mu <int>] Minimum population size. Defaults to 25                                                                            
[-lambda <int>] Number of solutions created before reaching the maximum population size (i.e., generation size). Defaults to 40
[-nbElite <int>] Number of elite individuals. Defaults to 5                                                                    
[-nbClose <int>] Number of closest solutions/individuals considered when calculating diversity contribution. Defaults to 4     
[-nbIterPenaltyManagement <int>] Number of iterations between penalty updates. Defaults to 100
[-targetFeasible <double>] target ratio of feasible individuals between penalty updates. Defaults to 0.2
[-penaltyIncrease <double>] penalty increase if insufficient feasible individuals between penalty updates. Defaults to 1.2
[-penaltyDecrease <double>] penalty decrease if sufficient feasible individuals between penalty updates. Defaults to 0.85
```

There exist different conventions regarding distance calculations in the academic literature.
The default code behavior is to apply integer rounding, as it should be done on the X instances of Uchoa et al. (2017).
To change this behavior (e.g., when testing on the CMT or Golden instances), give a flag `-round 0`, when you run the executable.

The progress of the algorithm in the standard output will be displayed as:

``
It [N1] [N2] | T(s) [T] | Feas [NF] [BestF] [AvgF] | Inf [NI] [BestI] [AvgI] | Div [DivF] [DivI] | Feas [FeasC] [FeasD] | Pen [PenC] [PenD]
``
```
[N1] and [N2]: Total number of iterations and iterations without improvement
[T]: CPU time spent until now
[NF] and [NI]: Number of feasible and infeasible solutions in the subpopulations 
[BestF] and [BestI]: Value of the best feasible and infeasible solution in the subpopulations 
[AvgF] and [AvgI]: Average value of the solutions in the feasible and infeasible subpopulations 
[DivF] and [DivI]: Diversity of the feasible and infeasible subpopulations
[FC] and [FD]: Percentage of naturally feasible solutions in relation to the capacity and duration constraints
[PC] and [PD]: Current penalty level per unit of excess capacity and duration
```

