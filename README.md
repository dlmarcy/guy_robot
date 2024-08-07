# guy_robot

## micro ros
### micro ros agent

go to the teensy with Arduino tutorial on the microros page  
make a micro ro worksapce and clone the respository  
they use rosdep to make sure dependencies are installed  
then build the micro ros setup package   
run the script to make the microros agent  
then run the script to build the micro ros agent  
the tutorial will show you how to run the agent  

## Arduino
### ide

Download the ide zip file and extract it to an Arduino folder  
trust the executable and double click to run  
do the teensy stuff to add the board, url and board manager  
I added the udev rules and that's it.

### micro ros arduino

go to the micro ros arduino git repository  
download the latest zip folder v2.0.7-iron  
I installed one of the boards, just to make the libraries folder  
extract the microros library to the library folder  
add our MircroROSArduino library to the library folder  
I tried to compile the microros arduino sketch but it failed  
open platform.txt in a hidden file:  
/home/robotguy/.arduino15/packages/teensy/hardware/avr/1.57.3/platform.txt  
copy the platform_teensy.txt from the microros arduino repository into this  
close the Arduino ide, then re-open it.  The sketch compiled  

## ROS
### micro sim

### ros control

### ros nav

### ros camera
#### usb_cam
install the camera driver, there are many others, but this one worked
sudo apt install ros-iron-usb-cam
copy /opt/ros/iron/share/usb_cam/config/params_1.yaml 
to local params folder and update the parameters
especially change camera_info_url: "package://robot_camera/params/camera_info.yaml"
so it points to the local copy of the camera_info.yaml file
copy /opt/ros/iron/share/usb_cam/config/camera_info.yaml to the local directory
this file will be recomuted when te camera is calibrated

#### image_pipeline
sudo apt install ros-iron-image-pipeline

#### apriltags
sudo apt install ros-iron-apriltag-ros
