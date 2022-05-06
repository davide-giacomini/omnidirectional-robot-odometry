# Omnidirectional Robot odometry

This repository contains the first laboratory project for the [Robotics course 2021/2022](https://www4.ceda.polimi.it/manifesti/manifesti/controller/ManifestoPublic.do?EVN_DETTAGLIO_RIGA_MANIFESTO=evento&aa=2021&k_cf=225&k_corso_la=481&k_indir=T2A&codDescr=089013&lang=IT&semestre=2&idGruppo=4336&idRiga=271084) of the Polytechnic University of Milan.

The project is implemented in ROS, an open-source robotics middleware suit.

## Requirements

## Description

In this project we were required to read from a bag file some information and calculate the odometry and other parameters an omnidirectional robot. A semple is illustrated below.

<p align="left">
    <img src="assets/images/robot-model.png" style="height:150px;"/>
</p>

We were given:
- Wheels encoder state:
    - RPM (noisy)
    - Ticks (more accurate)
- Nominal robot parameters:
    - Wheel radius (*r*) = 0.07m (could be a bit off)
    - Wheel position along x (*l*) = 0.200m
    - Wheel position along y (*w*) = 0.169m
    - Gear ratio (*T*) = 5:1
    - Encoder resolution(*N*): 42 CPR (Counts Per Rev.) (could be a bit off)
- Ground truth (GT) pose of the robot (acquired with OptiTrack)

And we were required to:
- Compute odometry using appropriate kinematics
    - Compute robot linear and angular velocities `v`, `⍵` from wheel encoders
    - Compute odometry using both Euler and Runge-Kutta integration methods
        - Add ROS parameters for initial pose
    - Calibrate (fine-tune) robot parameters to match ground truth
- Compute wheel control speeds from `v`, `⍵`
- Add a service to reset the odometry to a specified pose **(x,y,θ)**
- Use dynamic reconfigure to select between the desired integration method

We created a packet which contains three different nodes. Each node corresponds to a source code file, and the sources are located [here](src/omnidirectional_robot_odometry/src), under the `src` folder inside the packet `omnidirectional_robot_odometry`. Each node is in charge of a different task, which is explained more in detail in the next sections.

## Compute robot velocities

Given the wheel speeds of the robot from the bags, we were required to compute the robot linear and angular velocities. As a reference, we considered the formulas developed by [Taheri et al.](#1) for a mecanum wheeled mobile robot.

In particular, we are interested in the formulas of equation 22, 23 and 24 in the paper, which we report below:
- v<sub>x</sub>(t) = ( &#623;<sub>fl</sub> + &#623;<sub>fr</sub> + &#623;<sub>rl</sub> + &#623;<sub>rr</sub> ) * r&#47;4
- v<sub>y</sub>(t) = ( - &#623;<sub>fl</sub> + &#623;<sub>fr</sub> + &#623;<sub>rl</sub> - &#623;<sub>rr</sub> ) * r&#47;4
- &#623;<sub>z</sub>(t) = ( - &#623;<sub>fl</sub> + &#623;<sub>fr</sub> - &#623;<sub>rl</sub> + &#623;<sub>rr</sub> ) * r &#47;( 4(l<sub>x</sub> + l<sub>y</sub>) )

Where, in our case, the subscripts refer to the position of the wheel (*fl* = front left, *rr* = rear right, etc.).

The node [compute_velocities](src/omnidirectional_robot_odometry/src/compute_velocities.cpp) takes the wheel speeds published in the topic `wheel_states` by the bags, and then computes the angular and linear velocities using the aformentioned formulas. Considering that we were required to use the ticks and not the RPM, we needed to convert the ticks of the topic `wheel_states` in angular velocity of each wheel... 


## Getting Started




## Authors

- Davide Giacomini ([GitHub](https://github.com/davide-giacomini), [Linkedin](https://www.linkedin.com/in/davide-giacomini/), [email](mailto://giacomini.davide@outlook.com))
- Giuseppe Cerruto ([GitHub](https://github.com/GiuseppeCerruto))

## References

<a id="1">[1]</a> 
Taheri, Hamid, Bing Qiao, and Nurallah Ghaeminezhad. "Kinematic model of a four mecanum wheeled mobile robot." International journal of computer applications 113.3 (2015): 6-9.