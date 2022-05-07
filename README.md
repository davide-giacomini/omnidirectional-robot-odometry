# Omnidirectional Robot odometry

This repository contains the first laboratory project for the [Robotics course 2021/2022](https://www4.ceda.polimi.it/manifesti/manifesti/controller/ManifestoPublic.do?EVN_DETTAGLIO_RIGA_MANIFESTO=evento&aa=2021&k_cf=225&k_corso_la=481&k_indir=T2A&codDescr=089013&lang=IT&semestre=2&idGruppo=4336&idRiga=271084) of the Polytechnic University of Milan.

The project is implemented in ROS, an open-source robotics middleware suit.

## Requirements

Knowing ROS

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

[comment]: < special entity html codes: https://www.science.co.il/internet/html/Greek-characters.php, https://www.toptal.com/designers/htmlarrows/math/, and others sites like >
In particular, we are interested in the equations 22, 23 and 24 of the paper, which we report below:
- v<sub>x</sub>(t) = ( &omega;<sub>fl</sub> + &omega;<sub>fr</sub> + &omega;<sub>rl</sub> + &omega;<sub>rr</sub> ) &#8729; <sup>r</sup> &#8725; <sub>4</sub>
- v<sub>y</sub>(t) = ( - &omega;<sub>fl</sub> + &omega;<sub>fr</sub> + &omega;<sub>rl</sub> - &omega;<sub>rr</sub> ) &#8729; <sup>r</sup> &#8725; <sub>4</sub>
- &omega;<sub>z</sub>(t) = ( - &omega;<sub>fl</sub> + &omega;<sub>fr</sub> - &omega;<sub>rl</sub> + &omega;<sub>rr</sub> ) &#8729; <sup>r</sup> &#8725; <sub>4(l<sub>x</sub> + l<sub>y</sub>)</sub>

Where, in our case, the subscripts refer to the position of the wheel (*fl* = front left, *rr* = rear right, etc.).

The node "[compute_velocities](src/omnidirectional_robot_odometry/src/compute_velocities.cpp)" takes the wheel ticks published in the topic `wheel_states` by the bags, and then easily computes the angular velocity of each speed by using this equation:

&omega; = ( <sup>&Delta;<sub>ticks</sub></sup> &#8725; <sub>&Delta;<sub>time</sub></sub> ) &#8729; ( <sup>1</sup> &#8725; <sub>N</sub> ) &#8729; ( <sup>1</sup> &#8725; <sub>T</sub> ) &#8729; 2&pi;

Where <sup>&Delta;<sub>ticks</sub></sup> &#8725; <sub>&Delta;<sub>time</sub></sub> refers to the difference between the number of ticks divided by the period of time that have been measured. Once computing each angular velocity, the robot velocities can be computed using the aformentioned three equations, and they are eventually published as topic `cmd_vel`.

## Compute odometry

For this task, we were required to compute the robot odometry starting from the velocities we just calculated in the previous task. Furthermore, we were required to use both Euler and Runge-Kutta integration methods.

After computing the odometry, we published it as topic `odom` and wrote a TF broadcaster having as father the frame `odom` and as children the frame `base_link` (`odom` &#10132; `base_link`), which is basically the Reference System of the center of gravity of the robot. 

All of this can be found in the node "[compute_odometry](src/omnidirectional_robot_odometry/src/compute_odometry.cpp)", which reads from the topic `cmd_vel` the velocities.

### Euler integration method

We used as a reference the set of slides of Professor Matteucci [[2]](#2). From now on we will consider our `odom` frame as the fixed frame of the slides, and our `base_link` frame as the mobile frame of the robot. In case of a *differential drive* (slide 22), it can be seen that we have the x axis of the `base_link` frame always parallel to the direction of the velocity of the robot. This is not true for a *mecanum wheeled robot*, where we had to take into account both the parallel and perpendicular component of the velocity.

At slide number 27, the Euler integration method for *differential drive* is illustrated. We can see that v<sub>k</sub>cos&theta;<sub>k</sub> = v<sub>x</sub>(k) and v<sub>k</sub>sin&theta;<sub>k</sub> = v<sub>y</sub>(k), where v<sub>x</sub> and v<sub>y</sub> are the velocities relative to the `odom` reference frame. We could then generalize the Euler integration method in order to use it for the omnidirectional robot. The generalized equations can be recapped in:
- x<sub>k+1</sub> = x<sub>k</sub> + v<sub>x<sub>k</sub></sub>&#8729;&Delta;T
- y<sub>k+1</sub> = y<sub>k</sub> + v<sub>y<sub>k</sub></sub>&#8729;&Delta;T
- &theta;<sub>k+1</sub> = &theta;<sub>k</sub> + &omega;<sub>k</sub>&#8729;&Delta;T

In case of a *mecanum wheeled robot*, v<sub>x</sub> and v<sub>y</sub> have the perpendicular components too. In the image below you can see the `base_link` as the system reference &eta; / &tau; and the `odom` frame as the system reference x / y , as in the slides.

<p align="left">
    <img src="assets/images/euler-drawing.jpg" style="height:250px;"/>
</p>

With a bit of Euclidean Geometry, it is easy to obtain:
- v<sub>x<sub>k</sub></sub> = v<sub>&tau;<sub>k</sub></sub>cos&theta;<sub>k</sub> - v<sub>&eta;<sub>k</sub></sub>sin&theta;<sub>k</sub>
- v<sub>y<sub>k</sub></sub> = v<sub>&tau;<sub>k</sub></sub>sin&theta;<sub>k</sub> + v<sub>&eta;<sub>k</sub></sub>cos&theta;<sub>k</sub>

Applying these equations to the above ones, we can obtain the odometry starting from the components of the velocities relative to the `base_frame` of our omnidirectional robot.

### Runge-Kutta integration method

Starting from the assumptions made in the section before, we took as a reference the equations for *differential drive* listed at slide 28 ([[2]](#2)). In case of a *mecanum wheeled robot*, we split again the two components. It is illustrated in the image below.

<p align="left">
    <img src="assets/images/runge-kutta-drawing.jpg" style="height:250px;"/>
</p>

Putting all together, the three equations for Runge-Kutta are:
- x<sub>k+1</sub> = x<sub>k</sub> + v<sub>&tau;<sub>k</sub></sub>cos(&theta;<sub>k</sub> + <sup>&omega;<sub>k</sub>&Delta;T</sup> &#8725; <sub>2</sub>)&#8729;&Delta;T - v<sub>&eta;<sub>k</sub></sub>sin(&theta;<sub>k</sub> + <sup>&omega;<sub>k</sub>&Delta;T</sup> &#8725; <sub>2</sub>)&#8729;&Delta;T
- y<sub>k+1</sub> = y<sub>k</sub> + v<sub>&tau;<sub>k</sub></sub>sin(&theta;<sub>k</sub> + <sup>&omega;<sub>k</sub>&Delta;T</sup> &#8725; <sub>2</sub>)&#8729;&Delta;T - v<sub>&eta;<sub>k</sub></sub>cos(&theta;<sub>k</sub> + <sup>&omega;<sub>k</sub>&Delta;T</sup> &#8725; <sub>2</sub>)&#8729;&Delta;T
- &theta;<sub>k+1</sub> = &theta;<sub>k</sub> + &omega;<sub>k</sub>&#8729;&Delta;T

## ROS parameters for initial pose

We were required to add ROS parameters for defining the initial pose of the robot (x, y, &theta;). You can find them in the [launch file](src/omnidirectional_robot_odometry/launch/odom.launch), at line 3-4-5:

```
<param name="init_pose_x" value="0"/>
<param name="init_pose_y" value="0"/>
<param name="init_pose_th" value="0"/>
```

Those values are then used for initializing the initial pose in the [compute_odometry](src/omnidirectional_robot_odometry/src/compute_odometry.cpp) node.

Notice that the initial pose values are referred to the `odom` frame. Considered that at the beginning of the odometry, `base_link` overlaps `odom`, those values are put to zero.

## TF tree

In the [launch file](src/omnidirectional_robot_odometry/launch/odom.launch) you can also find three different TF static transforms `world` &#10132; `odom`. This is because the topic `robot/pose` of the bags refers to the frame `world`, hence we implemented a static translation taking into account the first values of the ground truth. In this way, our odometry better overlaps the GT odometry when visualized on `rviz`. There are three different transforms because each bag starts from a different position `world`.

Below we show the structure of the final TF tree:

<p align="left">
    <img src="assets/images/TF-tree.png" style="height:250px;"/>
</p>

## Getting Started




## Authors

- Davide Giacomini ([GitHub](https://github.com/davide-giacomini), [Linkedin](https://www.linkedin.com/in/davide-giacomini/), [email](mailto://giacomini.davide@outlook.com))
- Giuseppe Cerruto ([GitHub](https://github.com/GiuseppeCerruto))

## References

<a id="1">[1]</a> 
Taheri, Hamid, Bing Qiao, and Nurallah Ghaeminezhad. "Kinematic model of a four mecanum wheeled mobile robot." International journal of computer applications 113.3 (2015): 6-9.

<a id="2">[2]</a>
Matteo Matteucci. "Robot Localization - Wheels odometry", http://chrome.ws.dei.polimi.it/images/3/3e/Robotics_03_2021_Odometry.pdf