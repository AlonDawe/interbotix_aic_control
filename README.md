# interbotix_aic_control

This repository contains code developed for my MSc thesis project focused on enhancing the Active Inference Controller (AIC) for robotic manipulation tasks. Specifically, a modified version of the AIC, called the Reactive Active Inference Controller (ReAIC), was implemented and evaluated on an Interbotix PincherX 150 5-DOF manipulator using ROS (Robot Operating System). Additionally, several other controllers, including a PID controller and an Adaptive Friction Compensator (AFC) were implemented for comparison.

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

interbotix_aic_control/
│
├── src/
│   ├── pid_controller.py
│   ├── aic_controller.py
│   ├── reaic_controller.py
│   ├── afc_controller.py
│   ├── uaic_controller.py
│   └── ...
│
├── scripts/
│   ├── run_experiment_1.sh
│   ├── run_experiment_2.sh
│   ├── ...
│   └── ...
│
├── data/
│   ├── experiment_1_results.csv
│   ├── experiment_2_results.csv
│   └── ...
│
├── README.md
└── LICENSE

## Usage

1. Clone the repository:


