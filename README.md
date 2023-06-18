# MultiROS: ROS-Based Robot Simulation Environment for Concurrent Deep Reinforcement Learning

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

MultiROS is an open-source Robot Operating System (ROS)-based simulation environment designed for concurrent deep reinforcement learning. It provides a flexible and scalable framework for training and evaluating reinforcement learning agents in complex robotic tasks.

MultiROS supports training robots in both simulation and the real world. The simulation environment is based on Gazebo, providing a realistic simulation platform for testing and developing reinforcement learning algorithms. Additionally, MultiROS provides interfaces and tools for seamlessly transferring learned policies to real-world robotic systems.
## Prerequisites

Before installing MultiROS, make sure you have the following prerequisites:

### ROS Installation

MultiROS requires a working installation of ROS. If you haven't installed ROS yet, please follow the official [ROS installation guide](http://wiki.ros.org/ROS/Installation) for your specific operating system. MultiROS has been tested with ROS Noetic version, and the following instructions will guide you through the installation of ROS Noetic on Ubuntu 20.04:
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
### Other Packages 
MultiROS also requires the following additional packages:
- XTerm for terminal emulation:
    ```shell
    sudo apt install xterm
    ```
- MoveIt for motion planning:
    ```shell
    sudo apt install ros-noetic-moveit
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

You are now ready to proceed with the installation and usage of MultiROS.

Please note that the instructions assume you are using Ubuntu 20.04 and ROS Noetic. If you are using a different operating system or ROS version, make sure to adapt the commands accordingly.

## Installation

To get started with MultiROS, follow these steps:

1. Clone the repository:
    ```shell
    cd ~/catkin_ws/src
    git https://github.com/ncbdrck/multiros.git
    ```

2. MultiROS relies on several Python packages. You can install them by running the following command:

    ```shell
    cd ~/catkin_ws/src/multiros
    pip install -r requirements.txt
    ```
3. Build the ROS packages and source the environment:
    ```shell
   cd ~/catkin_ws/
   rosdep install --from-paths src --ignore-src -r -y
   catkin build
   source devel/setup.bash
    ```

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

## Contact

For questions, suggestions, or collaborations, feel free to reach out to us at [j.kapukotuwa@research.ait.ie](mailto:j.kapukotuwa@research.ait.ie).
