# ROS_Project(RR Robot)
#### Using ROS to develop software for the motion of a robotic arm, which consists of two Revolute Joints, can be controlled in two ways. The first method utilizes a Potentiometer to control the rotation of Servomotor Axis 1 and an Encoder to control the rotation of Servomotor Axis 2 The second method controls the rotation of both servo motors using a GUI, with RViz displaying a 3D model of the robot's operation.

#### Software Requirement :

1. Ubuntu 20.04
2. ROS Noetic
3. Rosserial Arduino Library 0.7.9
4. Python3
5. Arduino IDE 1.8.15 
6. Tkinter GUI python library
  
#### Hardware requirement:

1. 1 Arduino UNO R3  
2. 1 Potentiometer B 1K 
3. 1 Rotary Encoder KY-040
4. 1 MG996R Servo Motor
5. 1 MG90S Servo Motor
6. 1 Breadboard
7. 1 USB V2.0 (Type A To Type B )
8. Jump Wire
9. Power Supply 5V 2A
    
#### Circuit Diagram
![circuit](https://github.com/kcynn/ROS_Project/assets/154345247/102901e1-ca4a-400a-b6b9-e6303d7d294b)

### How To Install Ubuntu 20.04
####     Can be installed following the attached link >> https://www.youtube.com/watch?v=C5deqtXrpgk

### How To install of ROS Noetic in Ubuntu 
**1. Open Terminal Window**

> To perform the installation of ROS Noetic we need a command terminal and sudo user access. So, run the terminal app on your Ubuntu either by using the keyboard shortcut **Ctrl+ALT+T** or from all applications.

**2. Setup your sources.list**

> Setup your computer to accept software from packages.ros.org
```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
**3. Set up your keys**
```bash
    sudo apt install curl
```
```bash
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

**4. Installation**

> First, make sure your Debian package index is up-to-date
```bash
    sudo apt update
```
> Use the following command install ROS on Ubuntu
```bash
    sudo apt install ros-noetic-desktop-full
```

**5. Environment setup**


> You must source this script in every **bash** terminal you use ROS in.
```bash
    source /opt/ros/noetic/setup.bash
```
**6. Dependencies for building packages**
> Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately. For example, rosinstall is a frequently used command-line tool that enables you to easily download many source trees for ROS packages with one command.

> To install this tool and other dependencies for building ROS packages, run: 

```bash
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```
> #### Initialize rosdep

> Before you can use many ROS tools, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS. If you have not yet installed rosdep, do so as follows.
```bash
  sudo apt install python3-rosdep
```
> With the following, you can initialize rosdep.
```bash
  sudo rosdep init
  rosdep update
```



