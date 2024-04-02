# Active Inference Control for an Interbotix PincherX 150 Robotic Manipulator 

This repository contains code developed for my MSc thesis project focused on enhancing the Active Inference Controller (AIC) developed in [1] for robotic manipulation tasks. Specifically, a modified version of the AIC, called the Reactive Active Inference Controller (ReAIC), was implemented and evaluated on an [Interbotix PincherX 150](https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/px150.html) 5-DOF manipulator using ROS (Robot Operating System). Additionally, several other controllers, including the original AIC [1], the classic PID controller and an Adaptive Friction Compensator (AFC) [2] were implemented for comparison.

## Abstract

The application of the Free Energy Principle (FEP) and Active Inference (AIF) in robotics is an emerging field, with prior implementations primarily focused on demonstrating its viability. While these prior works have shown great promise in functionality, they have often lacked sufficient attention to the tunability of these systems for addressing specific performance criteria. This paper introduces the ReAIC, a modified prior implementation of the Active Inference Controller (AIC). The modification enhances the controller's ability to influence performance criteria such as Integral of Time-weighted Absolute Error (ITAE), settling time, and overshoot. A comprehensive tuning procedure is outlined alongside a detailed analysis revealing the performance impact of each tuning parameter. Comparative evaluations against three distinct controllers — the classic PID controller, a standard implementation of an AIC, and an Adaptive Friction Compensator (AFC) — are conducted across five experimental scenarios encompassing step response tests on a per-joint-basis of a physical 5-DOF robot arm. Results demonstrate the ReAIC's superior performance in gravity compensation tasks, exhibiting robust adaptability in the presence of dynamic environmental factors. Furthermore, comparative assessments reveal the ReAIC's sensitivity to induced friction, highlighting the AFC's superior performance under such conditions. This paper highlights the ReAIC's potential as a responsive and adaptable control framework, improving upon a previously implemented AIC.

## Controllers Implemented

- **PID Controller**
- **AIC Controller**
- **ReAIC Controller**
- **Adaptive Friction Compensator (AFC)**
- **Unbiased Active Inference Controller (u-AIC)**

## Repository Structure

TEMPLATE ONLY

``` bash
interbotix_aic_control/
│
├── src/
│   ├── AFC_controller.cpp
│   ├── AIC_controller.cpp
│   ├── PID_controller.cpp
│   ├── ReAIC_controller.cpp
│   ├── uAIC_controller.cpp
│   ├── OL_step_response.cpp
│   └── classes/
│       ├── AFC.cpp
│       ├── AIC.cpp
│       ├── PID.cpp
│       ├── ReAIC.cpp
│       └── uAIC.cpp
│
├── include/
│   ├── AFC.h
│   ├── AIC.h
│   ├── PID.h
│   ├── ReAIC.h
│   ├── uAIC.h
│
├── config/
│   ├── AFC_tuning.yaml
│   ├── AIC_tuning.yaml
│   ├── PID_tuning.yaml
│   ├── ReAIC_tuning.yaml
│   └── uAIC_tuning.yaml
│    
├── launch/
│   ├── AFC_control.launch
│   ├── AIC_control.launch
│   ├── PID_control.launch
│   ├── ReAIC_control.launch
│   ├── uAIC_control.launch
│   └── Step_control.launch
│
├── bagfiles/
│   ├── tuning/
│   ├── waist/
│   ├── wrist_ang/
│   └── wrist_rot/
│
├── README.md
└── LICENSE
```

## Setup
All experiments were executed using [Ubuntu 20.04](https://releases.ubuntu.com/focal/) OS and [ROS Noetic](https://wiki.ros.org/noetic).

Create a ROS workspace by navigating to the desired directory where you will store all your code, then execute the following:

```bash
source /opt/ros/noetic/setup.bash
```

```bash
mkdir ./interbotix_pincherX_ws/src
cd ./interbotix_pincherX_ws/
$ catkin_make
```

```bash
source devel/setup.bash
```

### Setting-up [Interbotix X-Series Arm Packages](https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms):
Please consult the Trossen Robotics [installation guide](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface.html) for your specific setup. Make sure to save all packages within the `src/` directory of your workspace.

Additionally you will need the following package that contains support level ROS wrappers and robot interface modules:

```bash
git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git
```

### Clone this repository:
In the `src/` directory of yout workspace clone this repository:

```bash
git clone https://github.com/AlonDawe/interbotix_aic_control.git
```

### Modify default config modes:

Navigate to `interbotix_xsarm_control/config/modes.yaml` and change `operating_mode: pwm`.
Navigate to `interbotix_xsarm_control/config/px150.yaml` and change `update_rate: 1000` in order to allow the joint states publishe to update at a rate of 1 kHz.

### Compile your workspace

Navigate to your workspace directory and run the following:

```bash
$ catkin_make
```






## References

[1]: C. Pezzato, R. Ferrari, and C. H. Corbato, “A Novel Adaptive Controller for Robot Manipulators Based on Active Inference,” IEEE Robotics and Automation Letters, vol. 5, pp. 2973–2980, Apr. 2020.

[2]: K. Verbert, R. Toth, and R. Babuska, “Adaptive Friction Compensation: A Globally Stable Approach,” IEEE/ASME Transactions on Mechatronics, pp. 1–1, 2015.


