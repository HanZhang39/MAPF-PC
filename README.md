# MAPF with Precedence Constraints

This repo contains the implementations of CBS-PC and PBS-PC in our MAPF with Precedence Constraints paper. Please refer to the paper for more details about the problem formulation and algorithm details.

To begin with, you can compile the project using the following commands

``` shell
cmake .
make
```
, and Cmake will generate three executables in the `bin` folder.
They are:

1. `cbs` runs the CBS-PC algorithm.
2. `pbs` runs the PBS-PC algorithm.
3. `task_assignment`  assigns tasks first and then  run a MAPF-PC algorithm. This is the one we used in the experiments.

For each executables, you can type `--help` to see the expected input arguments it. 
You can find example in `run_exp.sh`.
The `input_sample` folder contains sample input files.
