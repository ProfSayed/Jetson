# Cylinders Inspection using JetsonNano  
## Compile OpenCV from source on Jetson Nano Jetpack 4.6.2 [L4T 32.7.2]   
## on Jetson Nano with the latest opencv [4.5.5] with Gstreamer and CUDA
Remove any Installed OpenCV:  
```bash
sudo apt purge libopencv-dev libopencv-python libopencv-samples libopencv*  
python3 -m pip uninstall opencv-python
```  
## Required libraries for build  
```bash  
sudo apt install build-essential cmake git pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-22-dev
```  
## NOTES
1. opencv & opencv_contrib should be the same version/tag/tree 
2. make -j4 on usb-drive and at 100% it will crash > make with one processor 
## Clone the opencv repo
```bash  
mkdir ~/opencv_build && cd ~/opencv_build
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd ~/opencv_build/opencv
mkdir build && cd build  
```  
``` bash  
cmake -D WITH_CUDA=ON \
    -D ENABLE_PRECOMPILED_HEADERS=OFF \
    -D WITH_EIGEN=OFF \
    -D WITH_LIBV4L= ON \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_opencv_python2=ON \
    -D BUILD_opencv_python3=ON \
    -D PYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
    -D PYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
    -D WITH_GSTREAMER=ON .. \
    -D WITH_GTK=ON \
    -D WITH_QT=OFF \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
```
```bash 
make -j$(nproc)
sudo make install
sudo ldconfig
```
  
## Yolov5
```bash
cd ~  
```
[Follow the NEW official tutorial](https://github.com/ultralytics/yolov5/issues/9627)  
### Convert to TensorRT  
python3 export.py --weights yolov5s.pt --include engine --device 0  
## Setup the environment  
```bash  
sudo pip3 install rospkg catkin_pkg
sudo pip3 install --upgrade adafruit_blinka
sudo apt-get install ros-melodic-image-transport ros-melodic-vision-msgs
sudo apt-get install ros-melodic-smach ros-melodic-smach-ros ros-melodic-executive-smach ros-melodic-smach-viewer
```
## Create the workspace and download the repo
```bash  
mkdir -p ~/catkin_ws/src  
cd ~/catkin_ws/src  
git clone git@github.com:ProfSayed/Jetson.git  
cd ~/catkin_ws/  
catkin_make  
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc  
source ~/.bashrc  
```
Make all python scripts executable in the directory in order to run them(Not Needed anymore)  
```bash  
cd ~/catkin_ws/src/Jetson  
find ./ -name *.py -exec chmod +x {} \;
```
# Running the System
## Initialize Full System
On the main board connected with the gpio and runs the detection server (Minimum>= 4GB of RAM)  
```bash  
roslaunch jetson_bringup bringup_master.launch  
```
On the other board connected with the camera (Minimum>= 2GB of RAM)  
```bash  
roslaunch jetson_bringup bringup_slave.launch  
```
## To run nodes individually  
1. Run roscore
```bash  
roscore  
```
2. In a new Shell: Initialize parameters  
```bash  
roslaunch jetson_bringup init_param.launch  
```
Now Run any node as you like:  
* Start Object Detection node  
```rosrun jetson_detection detect_server.py```  
* Start Pusher and Stopper Actuators Server  
```roslaunch jetson_bringup start_actuators.launch```  
* You can now Call Stopper and the Pusher (True or False) Actuators  
```rosservice call /pusher_action True```  
```rosservice call /stopper_action True```  
* Start Sensors for both the Pusher and Stopper   
```roslaunch jetson_bringup start_sensors.launch```  
* You can now listen to the sensor topics   
```rostopic echo /pusher_count ```  
```rostopic echo /stopper_count ```  


