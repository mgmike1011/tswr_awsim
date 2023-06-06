# TSwR Projekt - MPC, iLQR, Stanley, Pure Pursuit
## Description
A project implementing four controllers for a vehicle in the AWSIM environment using ROS2 Humble.

## Installation Guide
1. Enter the autoware container -> run_x86.sh 
2. cd ~/autoware/src
3. git clone https://github.com/mgmike1011/tswr_awsim
4. cd ..
5. colcon build --packages-select tswr_awsim
6. . install/setup.bash
## Controllers
**Run all comands inside docker container**
### iLQR

#### Installing additional required packages
```bash
pip install drake
```
1. Start AWSIM simulator
```console
foo@bar:~/autoware$ cd f1tenth_simulators/F1Tenth_v0.5
```
```console
foo@bar:~/autoware/f1tenth_simulators/F1Tenth_v0.5$ ./F1Tenth_v0.5.x86_64 
```
2. Start path publisher

```console
foo@bar:~/autoware$ ros2 run tswr_awsim path_publisher
```
3. Start iLQR planning algorithm

```console
foo@bar:~/autoware$ ros2 run tswr_awsim iLQR_controller
```

### Linearized Model Predictive Control:
Computing steering angle and acceleration (with some limitations) based on current pose and a reference trajectory. The controller uses linearized model of vehicle kinematics to predict future state. It solves a discrete-time algebraic Riccati equation (DARE) to obtain the optimal feedback gain matrix K, which is used to compute optimal control input. 

1. Start AWSIM simulator
```console
foo@bar:~/autoware$ cd f1tenth_simulators/F1Tenth_v0.5
```
```console
foo@bar:~/autoware/f1tenth_simulators/F1Tenth_v0.5$ ./F1Tenth_v0.5.x86_64 
```

2. Start Linearized Model Predictive Control algorithm:
```console
foo@bar:~/autoware$ ros2 launch tswr_awsim lin_MPC_launch.py
```

### Pure Pursuit:
Computing steering angle using vehicle kinematics (with wheelbase 0.33 m), reference path and lookahaead distance 0.7 m. Equations correspond to a vehicle model with reference point in the center of the rear axle. 

1. Start AWSIM simulator
```console
foo@bar:~/autoware/f1tenth_simulators/F1Tenth_v0.5$ ./F1Tenth_v0.5.x86_64 
```

2. Start Pure Pursuit algorithm:
```console
foo@bar:~/autoware$ ros2 launch tswr_awsim pure_pursuit_launch.py 
```

### Stanley
Computing steering angle using the heading error and cross-track error. In this method, the cross-track error is defined as the distance between the closest point on the path with the front axle of the vehicle.
1. Start AWSIM simulator
```console
foo@bar:~/autoware/f1tenth_simulators/F1Tenth_v0.5$ ./F1Tenth_v0.5.x86_64 
```
2. Start Stanley algorithm:
```console
foo@bar:~/autoware$ ros2 launch tswr_awsim stanley_launch.py 
```
## Additional literature
- [iLQR](https://jonathan-hui.medium.com/rl-lqr-ilqr-linear-quadratic-regulator-a5de5104c750)
- [iLQR](https://github.com/Bharath2/iLQR)
- [Pure Pursuit, Stanley](https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081)
- [Stanley](http://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf)

## Authors
- Agnieszka Piórkowska
- Miłosz Gajewski
- Maciej Mirecki
- Mikołaj Zieliński

Politechnika Poznańska 2023