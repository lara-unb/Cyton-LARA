##########################################################################
C:
##########################################################################

##########################################################################
C++:
##########################################################################
This library requires the EIGEN3 library.

To install and configure the library, run

1-) sudo apt-get install libeigen3-dev
2-) sudo ln -s /usr/include/eigen3/Eigen/ /usr/local/include/

##########################################################################
ROS:
##########################################################################

This package is wrapper of the C++ dq_robotics library, therefore
ANY REQUIREMENT OF THE C++ LIBRARY IS ALSO A ROS REQUIREMENT.

MAKE THE C++ FOLDER EXAMPLES TO BE SURE THE SYSTEM IS CORRECLY CONFIGURED
BEFORE PROCEDING TO THE ROS PACKAGE REQUIREMENTS.

In order to use the dq_robotics ROS package, IT IS recomended to download
ALL the repository (the package needs the C++ folder and the folder struc
ture). Even so, only the ROS folder should be added to the ROS_PACKAGE_
PATH.

To use, add to your .bashrc:
export ROS_PACKAGE_PATH=~/[dq robotics repository folder]/ROS:$ROS_PACKAGE_PATH
Add the package in the manifest.xml:
<depend dq_robotics />
Add the headers in your .cpp files:
#include<dq_robotics/DQ.h> etc

##########################################################################
MATLAB:
##########################################################################
In order to use the DQ_robotics, just copy the directory dq_robotics 
somewhere and include dq_robotics/matlab to your Matlab path. 

Although this library is completely standalone, some examples use 
Peter Corke's Robotic Toolbox, so you are advised to install it as well.


