<a id="top"/> 
#squirrel_calibration

This repository provides calibration routines for the Robotino's camera and its mounting position.

Technical Maintainer: [ipa-rmb](https://github.com/ipa-rmb/) (Richard Bormann, Fraunhofer IPA) - richard.bormann@ipa.fraunhofer.de

##Contents

1. <a href="#1--installation-requirements">Installation Requirements</a>
2. <a href="#2--execution">Execution</a>
3. <a href="#3--software-architecture">Software architecture</a>


## 1. Installation Requirements: <a id="1--installation-requirements"/> 

####Debian packages
The following packages have to be installed: boost, libopencv-dev, libpcl-all-dev

####Squirrel packages
This repository requires to have the basic driver layer of Robotino (robotino_bringup) running, which is located in the repository [squirrel_robotino](https://github.com/squirrel-project/squirrel_robotino).

####ROS packages
**Commonly**
The ROS packages dependencies can be installed with the command:
```
rosdep install --from-path squirrel_calibration -i -y
```
## 2. Execution: <a id="2--execution"/> 
Instructions for doing the calibration are provided in file `squirrel_calibration/robotino_calibration/readme.md` or at [http://wiki.ros.org/robotino_calibration](http://wiki.ros.org/robotino_calibration).

## 3. Software architecture <a id="3--software-architecture"/> 

camera_base_calibration: [camera_base_calibration](https://raw.githubusercontent.com/squirrel-project/squirrel_calibration/master/software_architecture/camera_base_calibration.png)

<a href="#top">top</a>
