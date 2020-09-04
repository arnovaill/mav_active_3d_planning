# mav\_active\_3d\_planning
**mav\_active\_3d\_planning** is a modular framework for online informative path planner (IPP) design. 
We provide a modular framework for creating, evaluating and employing primarily sampling based, receding horizon algorithms that optimize a gain while minimizing a cost. This repository is a forked version of the [original repository](https://github.com/ethz-asl/mav_active_3d_planning) and was adapted to use [FIESTA](https://github.com/arnovaill/FIESTA) instead of [Voxblox](https://github.com/ethz-asl/voxblox). 

For additional information please see the [wiki](https://github.com/ethz-asl/mav_active_3d_planning/wiki) of the original repository.

# Credits
## Paper and Video
If you find this package useful for your research, please consider citing our paper:

* Lukas Schmid, Michael Pantic, Raghav Khanna, Lionel Ott, Roland Siegwart, and Juan Nieto, "**An Efficient Sampling-based Method for Online Informative Path Planning in Unknown Environments**", in *IEEE Robotics and Automation Letters*, vol. 5, no. 2, pp. 1500-1507, April 2020

```bibtex
@ARTICLE{Schmid20ActivePlanning,
  author={L. {Schmid} and M. {Pantic} and R. {Khanna} and L. {Ott} and R. {Siegwart} and J. {Nieto}},
  journal={IEEE Robotics and Automation Letters},
  title={An Efficient Sampling-Based Method for Online Informative Path Planning in Unknown Environments},
  year={2020},
  volume={5},
  number={2},
  pages={1500-1507},
  keywords={Motion and path planning;aerial systems;perception and autonomy;reactive and sensor-based planning},
  doi={10.1109/LRA.2020.2969191},
  ISSN={2377-3774},
  month={April},
}
```

The planner presented in the paper is given in `active_3d_planning_app_reconstruction/cfg/planners/reconstruction_planner.yaml`.
A video of the approach is available [here](https://www.youtube.com/watch?v=lEadqJ1_8Do).


## Installation
Installation instructions for Linux.

**Prerequisites**

1. If not already done so, install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full is recommended).

2. If not already done so, create a catkin workspace with [catkin tools](https://catkin-tools.readthedocs.io/en/latest/):

```shell script
sudo apt-get install python-catkin-tools
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/melodic  # exchange melodic for your ros distro if necessary
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
```

**Installation**

1. Move to your catkin workspace: 
```shell script
cd ~/catkin_ws/src
```

2. Install system dependencies: 
```shell script
sudo apt-get install python-wstool python-catkin-tools
```

3. Download repo using a SSH key or via HTTPS: 
```shell script
git clone git@github.com:ethz-asl/mav_active_3d_planning.git # SSH
git clone https://github.com/ethz-asl/mav_active_3d_planning.git # HTTPS
```

4. Download and install the dependencies of the packages you intend to use.

   * **Full Install:** dependencies of **all** packages can be installed using rosinstall:
   ```shell script
   # system dependencies, replace melodic with your ros distro if necessary:
   sudo apt-get install ros-melodic-cmake-modules ros-melodic-control-toolbox ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink ros-melodic-geographic-msgs autoconf libyaml-cpp-dev protobuf-compiler libgoogle-glog-dev liblapacke-dev libgeographic-dev
   pip install future unrealcv

   # If you already intialized ws tool use 'wstool merge -t'
   wstool init . ./mav_active_3d_planning/mav_active_3d_planning_https.rosinstall # HTTPS
   wstool update
   ```
   * **Partial Install:** Install dependencies of the packages you intend to use ([listed above](#Dependencies)) and remove unwanted packages from `mav_active_3d_planning/package.xml` as well as their source folders.

5. Source and compile: 
```shell script
source ../devel/setup.bash
catkin build mav_active_3d_planning # Builds this package only
catkin build # Builds entire workspace, recommended for full install.
```
