# PX4 and MAVROS Installation

<details>
    <summary style="font-size: 18px; font-weight: bold;">Official Documentation</summary>

### PX4 Installation

https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html

### MAVROS Installation

https://docs.px4.io/main/en/ros/mavros_installation.html

### MAVROS Offboard control example (Python)

https://docs.px4.io/main/en/ros/mavros_offboard_python.html
</details>

<details>
    <summary style="font-size: 18px; font-weight: bold;">TL;DR</summary>

## The extra Steps you need to do
### 1. Install catkin_tools
https://catkin-tools.readthedocs.io/en/latest/installing.html


### 2. Modify .bashrc
https://docs.px4.io/main/en/ros/mavros_offboard_python.html#launching-your-script
將下面幾行放到你的 .bashrc 中，並且 setup.bash 要在 PX4 的四行命令之前，並且後面不能再 source 其他 workspace 的 setup.bash。
```
source ~/<你的工作目錄>/devel/setup.bash
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
```

### 3. 修改 offb_node.py 將 Shebang 移到最上方，並改成 python3

<details>
    <summary>修改後的 offb_node.py</summary>

```python=
#! /usr/bin/env python3
"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        local_pos_pub.publish(pose)

        rate.sleep()
```
    
</details>

官方的說明文件上未將 Shebang 置於程式碼最上方，導致執行時會發生錯誤，且 ubuntu 20.04 版本的 ```python``` 環境變數是未被設定的，應該將 Shebang 裡的 ```python``` 改成 ```python3```。

```#! /usr/bin/env python3``` 

這行指令就是 Shebang，用於指定該檔案的直譯器，可使該程式檔可如同一般的執行檔一樣直接呼叫。

> e.g., 直接在 terminal 打 ```./foo.py```，前面不必加 python。

### 4. Compile PX4 SITL and Gazebo Classic

```
make -C ./PX4-Autopilot/ px4_sitl gazebo-classic
```
</details>


[![Watch the tutorial on YouTube](https://img.youtube.com/vi/3CWNg_pJPFQ/0.jpg)](https://www.youtube.com/watch?v=3CWNg_pJPFQ)  
https://www.youtube.com/watch?v=3CWNg_pJPFQ


> This video starts after completing the "Run the `ubuntu.sh` to install" step.  
You can download the vdi file from my Google Drive [here](https://drive.google.com/file/d/1LMt0zbts0C5hIjb7dVVFDUKfM0ROCESi/view?usp=drive_link) if you want to follow along with the video. (The user password is "rvl".)

> You can check out this article on my HackMD (for better HTML syntax support).  
https://hackmd.io/@zjewp/px4

## 1. Install PX4

#### Download PX4 Source Code

```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

#### Run the `ubuntu.sh` to install

```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

#### Compile PX4 SITL and Gazebo Classic

```
DONT_RUN=1 make -C ./PX4-Autopilot/ px4_sitl gazebo-classic
```

## 2. Install catkin_tools
#### Add ROS apt repositories
```
sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros-latest.list' && \
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```
#### Install catkin-tools with apt-get

```
sudo apt-get update && sudo apt-get install -y python3-catkin-tools
```

## 3. Install MAVROS

#### Install MAVROS with apt-get (Binary Install)
```
sudo apt-get install -y ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
```

#### Install GeographicLib datasets
```
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    sudo bash ./install_geographiclib_datasets.sh && \
    rm -f ./install_geographiclib_datasets.sh
```

## 4. Run the MAVROS Offboard control example

    
#### Create Catkin Workspace
```
mkdir -p ~/poop_space/src
cd ~/poop_space
catkin init
wstool init src
```

#### Create the ROS Package
```
cd src
catkin_create_pkg offboard_py rospy
cd ..
catkin build
source devel/setup.bash
roscd offboard_py
mkdir scripts
cd scripts
touch offb_node.py
chmod +x offb_node.py
```
#### Edit `offb_node.py` with following code:
    
<details>
  <summary style="font-weight: bold;">offb_node.py (modified)</summary>
    
```python=
#! /usr/bin/env python3
"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        local_pos_pub.publish(pose)

        rate.sleep()
```
</details>

<details>
    <summary style="font-weight: bold;">Additional explanation</summary>

官方的說明文件上未將 Shebang 置於程式碼最上方，導致執行時會發生錯誤，且 ubuntu 20.04 版本的 ```python``` 環境變數是未被設定的，應該將 Shebang 裡的 ```python``` 改成 ```python3```。

```#! /usr/bin/env python3``` 

這行指令就是 Shebang，用於指定該檔案的直譯器，可使該程式檔可如同一般的執行檔一樣直接呼叫。

> e.g., 直接在 terminal 打 ```./foo.py```，前面不必加 python。
    
</details>

#### Create the ROS launch file 
```
roscd offboard_py
mkdir launch
cd launch
touch start_offb.launch
```
#### Edit `start_offb.launch` with following code:
```=
<?xml version="1.0"?>
<launch>
	<!-- Include the MAVROS node with SITL and Gazebo -->
	<include file="$(find px4)/launch/mavros_posix_sitl.launch">
	</include>

	<!-- Our node to control the drone -->
	<node pkg="offboard_py" type="offb_node.py" name="offb_node_py" required="true" output="screen" />
</launch>
```

## Modify your .bashrc

#### Add following lines to your `.bashrc`
```=
source ~/poop_space/devel/setup.bash
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
```

<details>
  <summary style="font-weight: bold;">(optional) Edit .bashrc with echo and ">>"</summary>
    
```=
echo -e "source ~/poop_space/devel/setup.bash \n\
    source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default \n\
    export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/PX4-Autopilot \n\
    export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic \n\
    export GAZEBO_MODEL_PATH=\${GAZEBO_MODEL_PATH}:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models" \
    >> ~/.bashrc
```
</details>

#### Source your `.bashrc`
    
```
source ~/.bashrc
```

## Launching your script
```
roslaunch offboard_py start_offb.launch
```

---

<details>
  <summary style="font-weight: bold;">Reference</summary>
    
PX4 Installation  
https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html

MAVROS Installation  
https://docs.px4.io/main/en/ros/mavros_installation.html

MAVROS Offboard control example (Python)  
https://docs.px4.io/main/en/ros/mavros_offboard_python.html
    
PX4 UAV uses Mavros in Gazebo-classic simulation  
https://github.com/FCWTW/SummerCourse/tree/main/Week%203/HW2/README.md

ERROR: cannot launch node of type [px4/px4]  
https://github.com/PX4/PX4-Autopilot/issues/14762

</details>