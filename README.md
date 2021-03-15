# multi_agent_mpc_gazebo

![Alt text](https://github.com/rishabhdevyadav/multi_agent_mpc_gazebo/blob/main/gif/ezgif.com-video-to-gif.gif)

Tutlebot3 "waffle_pi" has been modified by removing sensors like imu, camera, gps, laser and all.

## How To Run

Launch Gazebo
1. Terminal 1st:-
```bash
roslaunch turtlebot3_gazebo gazebo.launch
```

Launch Robots
2. Terminal 2nd:-
```bash
export TURTLEBOT3_MODEL="waffle_pi"
roslaunch turtlebot3_gazebo multi_turtlebot3.launch
```

Run MPC using python3 (python socket: SERVER)
3. Terminal 3rd:-
```bash
cd catkin_ws/
source devel/setup.zsh
roscd turtlebot3_gazebo
cd scripts
python3 MPC.py
```

Give command to robots in Gazebo (pthon socket: CLIENT)
4. Terminal 4th:-
```bash
cd catkin_ws/
source devel/setup.zsh
roscd turtlebot3_gazebo
cd scripts
python robots_command.py
```


Note: To rerun the MPC.py and robots_command.py kill previous python socket
```bash
kill -9 $(ps -A | grep python | awk '{print $1}')
```
