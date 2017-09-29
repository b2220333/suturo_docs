Installation: Setting up basic components
=================================================

This article provides a step-by-step guide to the basic setup needed to install and run the Caterros project components. 



General installation & setup
------------------------------

Install ROS
^^^^^^^^^^^^^^^^^^^
Install and setup ROS indigo on Ubuntu 14.04. Therefore, follow the instructions from http://wiki.ros.org/indigo/Installation/Ubuntu. Recommended is to install the package ros-indigo-desktop-full. 

Install additional packages
^^^^^^^^^^^^^^^^^^^
    1. Install needed tools:: 
    
        sudo apt-get install ros-indigo-catkin ros-indigo-rospack python-wstool
        
    2. Install rosjava::
    
        sudo apt-get install ros-indigo-rosjava ros-indigo-rosjava-*
        
    3. Install the PR2 navigation::
    
        sudo apt-get install ros-indigo-pr2-navigation
        
Setup your Workspace
^^^^^^^^^^^^^^^^^^^
    1. Prepare a catkin workspace for the project (see http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment for further information):: 
    
        mkdir -p ~/suturo_ws/src
        cd ~/suturo_ws/
        catkin_make
        source devel/setup.bash
    
    2. Prepare another catkin workspace for the project dependencies:: 
    
        mkdir -p ~/suturo_dep/src
        cd ~/suturo_dep/
        catkin_make
        source devel/setup.bash
  
  
Clone repositories
------------------------------    

Clone the git repositories of all the modules you need on your machine into the src folder of your project workspace. To run the whole project, each module is needed but they don't have to run on the same machine. The corresponding git command is::

    git clone <URL>
      
You can find all the URLs in the following table: 

+--------------+------------------------------------------+----------------------------------------------+
| Name         | SSH                                      | HTTPS                                        |
+--------------+------------------------------------------+----------------------------------------------+
| planning     | git@github.com:suturo16/planning.git     | https://github.com/suturo16/planning.git     |
+--------------+------------------------------------------+----------------------------------------------+
| perception   | git@github.com:suturo16/perception.git   | https://github.com/suturo16/perception.git   |
+--------------+------------------------------------------+----------------------------------------------+
| knowledge    | git@github.com:suturo16/knowledge.git    | https://github.com/suturo16/knowledge.git    |
+--------------+------------------------------------------+----------------------------------------------+
| manipulation | git@github.com:suturo16/manipulation.git | https://github.com/suturo16/manipulation.git |
+--------------+------------------------------------------+----------------------------------------------+
| suturo_msgs  | git@github.com:suturo16/suturo_msgs.git  | https://github.com/suturo16/suturo_msgs.git  |
+--------------+------------------------------------------+----------------------------------------------+

After you have set up the corresponding dependencies, you can build your workspace again::

    cd ~/suturo_ws/src
    catkin_make

The description of the installation of each module can be found in the corresponding chapter.

suturo_msgs
------------------------------ 
This module is needed by all the other modules, so clone it first into the src folder of your project workspace.


