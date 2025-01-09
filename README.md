# amr_navigation
it evaluates several local and global planner

### Clone
```
git clone https://github.com/labiybafakh/amr_navigation
```

### Build
#### Native environment
Make sure that all of the dependencies have been installed,
```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the package and soure it
```bash
catkin build -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

#### Docker
1. Config and run docker
```bash
chmod +x Docker/build_docker.sh Docker/run_docker.sh 
./Docker/build_docker.sh
./Docker/run_docker.sh
```
2. Source to the workspace
```bash
source devel/setup.bash
```


### Run
#### Footprint Options
- To use the normal configuration, just set the argument as false or without the argument since it is the default
```bash
roslaunch amr_navigation navigation.launch
```

- To use the angled robot configuration for costmap, the argument of use_angled should be set as true
```bash
roslaunch amr_navigation navigation.launch use_angled:=true
```

#### Planner options
Several local planner such as DWAPlanner(dwa), TrajectoryPlanner(tp), and TEBPlanner(teb) are utilized.

Not only the local planner, but the global planner NavfnPlanner(navfn), GlobalPlanner(gplanner) and ~~SPBL Lattice Planner(spbl)~~.

Default local and global planner are dwa and navfn if the arguments are not set.
```bash
roslaunch amr_navigation navigation.launch global_plan:=gplanner local_plan:=dwa
```