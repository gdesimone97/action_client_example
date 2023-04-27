# Trajectory

## Install

1. Into workspace directory run: ``` rosdep init ```; If an error occurs run: ``` sudo rosdep init ```
2. Run: ``` rosdep update ```
3. Into workspace directory run: ``` catkin build ```; ignore errors
4. Run ``` source devel/setup.bash ``` 
5. Run ``` rosdep install robot_pose_ekf ```
6. Run ``` catkin build ```
7. If an error occurs, re-run: ``` catkin build ```

## Description of the exercises
1. es -> ideal trajectory
2. es_noise -> trajectory with noise on odometry without corrections
3. es_mesure -> trajectory using sensors, LIDAR and IMU
4. es_ekf -> trajectory using the robot pose ekf node
5. es_kalman -> trajectory using the own Kalman Filter

## Run
1. ``` roslaunch es_2023 <exercise (see previous section)>.launch path_fname:=<line.csv or points.cvs> ```
2. Wait for gazebo start then press enter
Example:
    ``` roslaunch es_2023 es.launch path_fname:=points.csv ```
    ``` roslaunch es_2023 es_ekf.launch path_fname:=line.csv ```
