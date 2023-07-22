# ros2_rt_dualarm
## build
```bash
    cd ~/ros2_foxy
    colcon build --symlink-install --packages-select ros2_rt_dualarm
```
## start real robot
```bash
    ~/ros2_foxy/src/ros2_rt_dualarm/bin/ros2_rt_dualarm
```
## start real robot with rviz
```bash
    cd ~/ros2_foxy
    ros2 launch ros2_rt_dualarm dualarm.launch.py
```

## sim rviz start
```bash
    cd ~/ros2_foxy
    ros2 launch ros2_rt_dualarm dualarm_sim.launch.py
```
## start sim robot ros2
```bash
    ~/ros2_foxy/src/ros2_rt_dualarm/bin/ros2_rt_dualarm_ros2_sim
```

## start sim robot pybullet
```bash
    ~/ros2_foxy/src/ros2_rt_dualarm/bin/ros2_rt_dualarm_bullet_sim
```
