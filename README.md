# Setup the environment  
```bash  
sudo pip3 install rospkg catkin_pkg
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
# Running the System
## To run the full system  
```bash  
roslaunch jetson_bringup start_bringup.launch  
```
## To run nodes individually  
First: Initialize parameters  
```bash  
roslaunch jetson_bringup param_initializer.launch  
```
Now any node as you like:  
* Run Object Detection node
```rosrun jetson_detection detect_server.py```
* Run Pusher
```rosrun jetson_detection detect_server.py```
* Run Stopper

