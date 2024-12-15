# amr_navigation
it evaluates several local and global planner

### Clone
```
git clone https://github.com/labiybafakh/amr_navigation
```

### Build
Make sure that all of the dependencies have been installed,
```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the package and soure it
```bash
catkin build -DCMAKE_BUILD_TYPE=Release
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
Several local planner can be tested on the navigation.
- Dynamic Window Approach
```bash
roslaunch amr_navigation navigation.launch local:=dwa
```
- TEB
```bash
roslaunch amr_navigation navigation.launch local:=teb
```
- TrajectoryPlanner
```bash
roslaunch amr_navigation navigation.launch local:=tp
```