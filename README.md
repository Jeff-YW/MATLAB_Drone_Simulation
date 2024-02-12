# MATLAB Drone Simulation Project

## Overview
This project explores the dynamics and control of a quadcopter drone through simulations. It progresses from basic physical principles to advanced control mechanisms under various conditions.

## Visual Guide

### Drone Physics
![Basic Physics of a Drone](/vids_and_pics/quad_physics_demo.png)
*Figure 1: Basic physics principles governing drone flight.*

### Simulation Dynamics
![Drone Dynamics Simulation](/vids_and_pics/quad_sim_demo.png)
*Figure 2: Simulation showing drone dynamics with physical constraints and parameters.*

## Simulation Demos

### Question 1: Non-linear Model Behavior
[![Simulation Demo for Question 1](/vids_and_pics/video_thumbnail_q1.png)](/vids_and_pics/q1_drone_c.mp4)
*Video 1: Demonstrates the drone's behavior under different propeller speeds.*

### Question 2: Linear vs Non-linear Model Comparison
[![Simulation Demo for Question 2](/vids_and_pics/video_thumbnail_q2.png)](/vids_and_pics/q2_drone_c.mp4)
*Video 2: Compares the linearized model's behavior with the non-linear model.*

### Question 3a: Feedback Control with Perfect Sensors
[![Simulation Demo for Question 3a](/vids_and_pics/video_thumbnail_q3a.png)](/vids_and_pics/q3a_drone_c.mp4)
*Video 3: Showcases feedback control performance following a specified trajectory with perfect sensors.*

### Question 3b: Feedback Control with Sensor Noise
[![Simulation Demo for Question 3b](/vids_and_pics/video_thumbnail_q3b.png)](/vids_and_pics/q3b_drone_c.mp4)
*Video 4: Evaluates feedback control under the influence of Gaussian sensor noise.*

## Running the Simulations

- **For Question 1**: Run `quadcopter_script_q12.m` with `Drone_q1.m` enabled and `Drone_q2.m` disabled.
- **For Question 2**: Run `quadcopter_script_q12.m` with `Drone_q2.m` enabled and `Drone_q1.m` disabled.
- **For Question 3a**: Run `quadcopter_script_q3ab.m` with `Drone_q3a.m` enabled and `Drone_q3b.m` disabled.
- **For Question 3b**: Run `quadcopter_script_q3ab.m` with `Drone_q3b.m` enabled and `Drone_q3a.m` disabled.

Ensure to check the supporting files for Questions 2 and 3 for pre-generated matrices required for the simulations.

