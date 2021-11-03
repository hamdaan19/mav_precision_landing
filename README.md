# mav_precision_landing
This repository contains code for autonomous precision landing for micro aerial vehicles. It has been tested on ROS-Noetic. 

## Dependencies
1. [rotors_simulator](https://github.com/ethz-asl/rotors_simulator) by ETH-ASL
2. [opencv-contrib-python](https://pypi.org/project/opencv-contrib-python/)
3. [Klampt](http://motion.cs.illinois.edu/software/klampt/latest/pyklampt_docs/Manual-Installation.html)

## Getting Started
1. Install [rotors_simulator](https://github.com/ethz-asl/rotors_simulator) by following the instructions on the (linked) page.
2. If your local machine has opencv-python installed, then uninstall it and install [opencv-contrib-python](https://pypi.org/project/opencv-contrib-python/):
```
pip uninstall opencv
pip install opencv-contrib-python
```
3. Install Klampt from source by following the instructions [here] (https://github.com/krishauser/Klampt/blob/master/Cpp/docs/Tutorials/Install-Linux.md)
