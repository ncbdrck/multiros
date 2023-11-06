# MultiROS: ROS-Based Robot Simulation Environment for Concurrent Deep Reinforcement Learning

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

MultiROS is an open-source Robot Operating System ([ROS](http://wiki.ros.org/))-based simulation environment designed for concurrent deep reinforcement learning. It provides a flexible and scalable framework for training and evaluating reinforcement learning agents for complex robotic tasks.

This package supports training robots in both simulation and the real world. The simulation environment is based on [Gazebo](https://gazebosim.org/), providing a realistic simulation platform for testing and developing reinforcement learning algorithms. 

Additionally, MultiROS provides interfaces and tools for seamlessly transferring learned policies to real-world robotic systems.
## Prerequisites

Before installing MultiROS, make sure you have the following prerequisites:

### ROS Installation

MultiROS requires a working installation of ROS. If you haven't installed ROS yet, please follow the official [ROS installation guide](http://wiki.ros.org/ROS/Installation) for your specific operating system. This package has been tested with [ROS Noetic](http://wiki.ros.org/noetic) version, and the following instructions will guide you through the installation of ROS Noetic on Ubuntu 20.04:
```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```
### Catkin Tools
MultiROS uses Catkin as the build system for ROS packages. Install Catkin Tools by running the following command:
```shell
sudo apt-get install python3-catkin-tools
```

### Create ROS Workspace
Before using MultiROS, you need to create a ROS workspace to build and run your ROS packages. Follow these steps to create a workspace:
```shell
cd ~
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Other Packages 
MultiROS also requires the following additional packages:
- XTerm for terminal emulation:
    ```shell
    sudo apt install xterm
    ```
- [MoveIt](https://moveit.ros.org/) for motion planning:
    ```shell
    sudo apt install ros-noetic-moveit
    ```
- [kdl_parser_py](http://wiki.ros.org/kdl_parser_py) for parsing URDF files (for ros_kinematics):
    ```shell
    sudo apt install ros-noetic-kdl-parser-py
    ```
- trac_ik_python for inverse kinematics:
    ```shell
  # Download and install trac_ik_python
    cd ~/catkin_ws/src
    git clone https://bitbucket.org/traclabs/trac_ik.git
  
    # Build the package
    cd ~/catkin_ws
    rosdep install --from-paths src --ignore-src -r -y
    catkin build
    source devel/setup.bash
  ```

You are now ready to proceed with the installation and usage of MultiROS.

Please note that the instructions assume you are using Ubuntu 20.04 and ROS Noetic. If you are using a different operating system or ROS version, make sure to adapt the commands accordingly.

## Installation

To get started with MultiROS, follow these steps:

1. Clone the repository:
    ```shell
    cd ~/catkin_ws/src
    git clone https://github.com/ncbdrck/multiros.git
    ```

2. MultiROS relies on several Python packages. You can install them by running the following command:

    ```shell
    # Install pip if you haven't already by running this command
    sudo apt-get install python3-pip

    # install the required Python packages for MultiROS by running
    cd ~/catkin_ws/src/multiros
    pip3 install -r requirements.txt
    ```
3. Build the ROS packages and source the environment:
    ```shell
   cd ~/catkin_ws/
   rosdep install --from-paths src --ignore-src -r -y
   catkin build
   source devel/setup.bash
    ```
## Usage

Refer to the [templates](https://github.com/ncbdrck/multiros/tree/main/src/multiros/templates) or the [examples](https://github.com/ncbdrck/reactorx200_ros_reacher) to see how MultiROS can create a simulation environment for RL applications.

The installation instructions for the examples are provided in the respective repositories.

## License

MultiROS is released under the [MIT License](https://opensource.org/licenses/MIT). Please see the LICENSE file for more details.

## Acknowledgements

We would like to thank the following projects and communities for their valuable contributions, as well as the authors of relevant libraries and tools used in MultiROS.
- [ROS (Robot Operating System)](https://www.ros.org/)
- [Gazebo](https://gazebosim.org/)
- [Openai Gym](https://github.com/openai/gym/)
- [openai_ros](http://wiki.ros.org/openai_ros)
- [frobs_rl](https://frobs-rl.readthedocs.io/en/latest/)
- [gym-gazebo](https://github.com/erlerobot/gym-gazebo/)

## Cite

If you use MultiROS in your research or work and would like to cite it, you can use the following citation:

Repository
```bibtex
@misc{multiros,
  author = {Kapukotuwa, Jayasekara},
  booktitle = {GitHub repository},
  publisher = {GitHub},
  title = {MultiROS: ROS-Based Robot Simulation Environment for Concurrent Deep Reinforcement Learning},
  url = {https://github.com/ncbdrck/multiros},
  year = {2022}
}
```
Article
```bibtex
@inproceedings{kapukotuwa_multiros_2022,
	title = {{MultiROS}: {ROS}-{Based} {Robot} {Simulation} {Environment} for {Concurrent} {Deep} {Reinforcement} {Learning}},
	shorttitle = {{MultiROS}},
	doi = {10.1109/CASE49997.2022.9926475},
	booktitle = {2022 {IEEE} 18th {International} {Conference} on {Automation} {Science} and {Engineering} ({CASE})},
	author = {Kapukotuwa, Jayasekara and Lee, Brian and Devine, Declan and Qiao, Yuansong},
	month = aug,
	year = {2022},
	note = {ISSN: 2161-8089},
	pages = {1098--1103},
}
```

## Contact

For questions, suggestions, or collaborations, feel free to reach out to the project maintainer at [j.kapukotuwa@research.ait.ie](mailto:j.kapukotuwa@research.ait.ie).
