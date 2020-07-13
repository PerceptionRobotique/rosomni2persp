rosomni2persp: unwarping an omnidirectional image as a perpsective one

July 2019
Author: G. Caron
Contact: guillaume.caron@u-picardie.fr

Prerequisities
0. CMake (version 3.14.5 tested)
1. ROS (version Kinetic tested)
2. ViSP (version 3.2.0 tested)
3. libPeR_base (version 0.0.2 tested)

Configure and prepare rosomni2persp to build with catkin
0. be sure the rosomni2persp directory is in the src directory of your catkin workspace
1. export PER_DIR=/home/gcaron/Developpement/libPeR_base/build/
2. move to your catkin workspace and type catkin_make

Run rosomni2persp
1. Use one of the launch files of the launch directory to run omni2persp: from the catkin workspace directory (running a source devel/setup.bash command) run, for instance, the command (with updated possible bagfile path and imagestopiname, if needed): 
roslaunch ros_omni2persp omni2persp_gray.launch bagfile:=somebag.bag inputImagesTopic:=/camera/fisheye1/image_raw focalFact:=0.33

Note 1: to handle compressed/uncompressed images from source bag file, comment or uncomment the image_transport parameter of the omni2persp_gray.launch

