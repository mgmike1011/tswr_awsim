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