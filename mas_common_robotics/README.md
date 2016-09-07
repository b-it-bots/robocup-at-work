## Install Ubuntu
The repository and its related components have been tested under the following Ubuntu distributions:

- ROS Indigo: Ubuntu 14.04

If you do not have a Ubuntu distribution on your computer you can download it here

     http://www.ubuntu.com/download

## Git - Version Control
### Install Git Software
Install the Git core components and some additional GUI's for the version control:

     sudo apt-get install git-core gitg gitk git-gui

### Set Up Git
Now it's time to configure your settings. To do this you need to open a new Terminal. First you need to tell git your name, so that it can properly label the commits you make:

     git config --global user.name "Your Name Here"

Git also saves your email address into the commits you make.

     git config --global user.email "your-email@youremail.com"


### GIT Tutorial
If you have never worked with git before, we recommend to go through the following basic git tutorial:

     http://excess.org/article/2008/07/ogre-git-tutorial/


## ROS - Robot Operating System
### Install ROS
The repository has been tested successfully with the following ROS distributions. Use the link behind a ROS distribution to get to the particular ROS installation instructions.

- ROS Indigo - http://wiki.ros.org/indigo/Installation/Ubuntu

NOTE: Do not forget to update your .bashrc! 


### ROS Tutorials
If you have never worked with ROS before, we recommend to go through the beginner tutorials provided by ROS:

     http://wiki.ros.org/ROS/Tutorials

In order to understand at least the different core components of ROS, you have to start from tutorial 1 ("Installing and Configuring Your ROS Environment") till tutorial 7 ("Understanding ROS Services and Parameters"). 

## Set up a catkin workspace

    source /opt/ros/indigo/setup.bash
    mkdir -p ~/catkin_ws/src; cd ~/catkin_ws/src
    catkin_init_workspace
    cd ..
    catkin_make
    
## Clone and compile the MAS common robotics software
First of all you have to clone the repository.

    cd ~/catkin_ws/src;
    git clone gitgate@mas.b-it-center.de:mas-group/mas_common_robotics.git

Then go on with installing further external dependencies:
       
    cd ~/catkin_ws/src/mas_common_robotics
    ./repository.debs
    
    source ~/catkin_ws/devel/setup.bash

The last command should be added to the ~/.bashrc file so that they do not need to be executed everytime you open a new terminal.


And finally compile the repository:

    cd ~/catkin_ws
    catkin_make
    
If no errors appear everything is ready to use. Great job!
