# TSwR Projekt - MPC, iLQR, Stanley, Pure Pursuit

1. Wejście do kontenera -> run_x86.sh 
2. cd ~/autoware/src
3. git clone https://github.com/mgmike1011/tswr_awsim
4. cd ..
5. colcon build --packages-select tswr_awsim
6. . install/setup.bash
7. Test działania: ros2 run tswr_awsim lin_MPC_controller 
6. :)

## iLQR:

Run all comands inside docker

Install required packages (Nie wiem jak to dodać żeby to rosdep zainstalował w sumie ewnwtualnie możemy to tak tu zostawić)

```bash
pip install drake
```
Start path publisher

```python
ros2 run tswr_awsim path_publisher
```
Start iLQR planning algorithm

```python
ros2 run tswr_awsim iLQR_controller
```

## Linearized Model Predictive Control:
Run all comands inside docker container

Start AWSIM simulator
```console
foo@bar:~/autoware$ cd f1tenth_simulators/F1Tenth_v0.5
```
```console
foo@bar:~/autoware/f1tenth_simulators/F1Tenth_v0.5$ ./F1Tenth_v0.5.x86_64 
```

Start Linear Model Predictive Control algorithm:
```console
foo@bar:~/autoware$ ros2 launch tswr_awsim lin_MPC_launch.py
```

## Pure Pursuit:
Run all comands inside docker containter 

Start AWSIM simulator
```console
foo@bar:~/autoware/f1tenth_simulators/F1Tenth_v0.5$ ./F1Tenth_v0.5.x86_64 
```

Start Pure Pursuit algorithm:
```console
foo@bar:~/autoware$ ros2 launch tswr_awsim pure_pursuit_launch.py 
```



