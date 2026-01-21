Upload the .ino file into the ESP32 Node MCU.
Adjust the baudrate as needed.

run this in the docker script:
```
cd docker_run
bash docker_run.sh

source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash

apt-get update
apt-get install -y ros-humble-moveit-servo
apt install nano

pip install pyserial
pip install ikpy

ros2 run xacro xacro /opt/ros/humble/share/kortex_description/robots/gen3.xacro arm:=gen3 gripper:=robotiq_2f_85 dof:=7 vision:=true sim:=true > /colcon_ws/gen3.urdf

cd ../colcon_ws/src

nano kinova_teleop.py
```
# paste the code from the kinova_teleop.py in here. save and exit.
#ensure the baudrate matches the one from the .ino file

--------------------------------------------------
Run the following launch file in terminal 1:

```
ros2 launch kortex_bringup gen3.launch.py \
  robot_ip:=yyy.yyy.yyy.yyy \
  use_fake_hardware:=true
```

Run the python script to control the robot in terminal 2:

`python3 kinova_teleop.py`
