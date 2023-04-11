# quori_ros

This catkin workspace contains all of the ROS packages necessary for full operation of the Quori robot platform from UPenn.

## Prerequisites

The package is compatible with **Ubuntu 16.04 / ROS Kinetic** and **Ubuntu 20.04 / ROS Noetic**, built with the **Catkin** system. Make sure you have the right environment configured. This documentation assumes you are using noetic, so replace it with kinetic if needed.

## Initial Setup

1. Install Visual Studio Code through the Software Center

2. Install Github Desktop (optional, but nice to have)
```
sudo wget https://github.com/shiftkey/desktop/releases/download/release-3.1.1-linux1/GitHubDesktop-linux-3.1.1-linux1.deb
sudo apt-get install gdebi-core 
sudo gdebi GitHubDesktop-linux-3.1.1-linux1.deb
```

3. Follow the ROS Noetic Installation (http://wiki.ros.org/noetic/Installation/Ubuntu)

4. Clone Repository
```
mkdir quori_files
cd quori_files
git clone https://github.com/CMU-RASL/quori_ros.git
cd quori_ros
git submodule init
```

5. Add repository to Github Desktop (optional)

6. Install helper repositorities
```
sudo apt-get install ros-noetic-sound-play ros-noetic-rgbd-launch ros-noetic-libuvc-camera ros-noetic-libuvc-ros
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-control
sudo apt-get install python3-catkin-tools ros-noetic-smach-ros ros-noetic-smach-viewer
sudo apt-get install python3-pip
sudo apt-get install libglfw3 libglfw3-dev
sudo apt-get install ros-noetic-rqt-joint-trajectory-controller
sudo apt-get install python3-catkin-tools
rosdep update
rosdep install --from-paths src --ignore-src -r -y
pip install mediapipe
pip install fastdtw
```

7. Build the workspace
```
catkin config --init
catkin build
source devel/setup.bash
```

## Each new terminal
```
cd quori_files/quori_ros
source devel/setup.bash
```

## Set-up sound
Go to Settings in Ubuntu and make sure the output is set to Headphones and the volume is all the way up

## Run Robot/Simulation

- Terminal 1: `roslaunch quori_launch quori_robot_main.launch` (make sure you set the hardware tag in this file to true or false depending on whether you want to run the simulation or the robot)
- If you want to test the controllers:
    - Terminal 2: `rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller`
- Terminal 3: `cd src/quori_face_generator/gui && python3 -m http.server 8000` 
    - Open a browser at `http://localhost:8000/face_generator.html`

## Turn on the Robot

1. Flip the breaker
2. Make sure the inverter has the switch turned to |
3. Press the power button to turn PC power on




