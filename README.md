# Drone Control

The objective of this assignment is to create a simple feedback control algorithm for position stabilization of an Unmanned Aerial System (UAS). The alternate objective is to use advanced control techniques to improve the stabilization of the UAS in the presence of wind.

## With Wind
https://github.com/nathanshankar/drone-control/assets/66565433/420a624a-620d-4a44-900a-9ef6a6e56c4f

## Without Wind
https://github.com/nathanshankar/drone-control/assets/66565433/c26fdb9e-ec11-4855-871f-1565348f2d85

## Overview

From the block diagram, we can visualize the overall working of our code. Let’s review the working by each segment:
![image](https://github.com/nathanshankar/drone-control/assets/66565433/a3956ae3-2b0c-4d85-8e01-13f3818f1ad4)

1. **Controller Inputs and Outputs**:
    - The controller is fed with inputs of the state, the target position, and the time step.
    - It outputs the motor commands.

2. **Error Calculation and Integral Reset**:
    - From the controller’s inputs, we unpack the state and the target position and calculate the error in its current position relative to the target position.
    - We also check if the current target position is the same as the previous target position; if not, we set all the integral accumulators to 0.

## Disturbance Observer (DOB)

In control systems, disturbances are unexpected factors that disrupt system behavior, complicating performance goals. The Disturbance Observer (DOB) estimates and compensates for these disturbances, enhancing system robustness and stability. DOB employs a separate observer to estimate disturbance effects on system dynamics (with and without wind). By integrating this estimation into the control loop, the controller effectively mitigates disturbances, minimizing their impact on system performance. 

- In simulations with wind, the y-axis remains largely unaffected, while the x-axis is significantly influenced.

## Dynamic PID Function

We've implemented a dynamic PID function for attitude control, which adjusts PID coefficients based on the x-axis error magnitude. This adaptive approach enables precise control over the drone's attitude across different flight conditions.

- For small errors (<1), fine-tuning and precision are prioritized.
- For larger errors (>1), faster correction and stabilization are prompted.
- This adaptive strategy ensures robust attitude control despite external disturbances or operational variations.

## Integral Windup Mitigation

Integral windup, observed in simulations as overshoot or instability, is mitigated by constraining the integral accumulators' range. This prevents excessive error accumulation during transient conditions, maintaining better control performance.

## Wind Estimation and PID Cascade Loop

We utilize the DOBC outputs to estimate wind speed along the x-axis, feeding into a PID cascade loop. This loop's x-axis PID control calculates the target attitude, compared with the current attitude to compute the error and drone's angular velocity. This information drives the outer PID loop, determining torque. Attitude control PID gains are dynamically adjusted using the Dynamic PID function's outputs. Furthermore, y-axis wind speed, derived from the DOBC, influences the PID control loop managing the drone's upward thrust along the y-axis.

## Thrust and Torque Calculation

The calculated thrust and torque are combined using mixer theory equations. The resulting output passes through a throttle clamper to maintain it within the 0 to 1 range, serving as the controller function output.

## Performance Comparison

Comparing our controller's performance on the drone with and without wind, we observe a similar response, highlighting its robustness and adaptability across varying conditions. This underscores the effectiveness of leveraging advanced control algorithms.

## Tuning the System

To tune our system, we segmented our objective into four different parameters. These parameters are tuned using trial and error and from observations on the system dynamics.

1. **Y-Axis Tuning**:
    - We started by setting initial setpoints and using a high P gain to quickly decrease error and enhance speed.
    - A high integral gain was crucial for eradicating steady-state errors, while a high derivative gain allowed rapid responses to abrupt error changes, effectively damping oscillations and ensuring stability.
    - This method facilitated swift convergence along the y-axis with minimal oscillations.

2. **X-Axis PID Tuning within a Cascaded Control System**:
    - Iterative adjustments were made, initially using high gains which led to instability and drone toppling.
    - Cautious increases in integral gain improved wind response.
    - Despite observing oscillations during target approach, convergence was quicker compared to previous iterations.
    - Dynamic PID tuning prioritized expediting convergence during large x-axis errors and precision during lower errors, requiring a high derivative to counteract oscillations in the cascade loop.

## Addressing Oscillations and Fluid Trajectory

While our system performed adequately under standard conditions, it displayed fluid trajectory and oscillations, notably in strong gusty winds. To address this, we integrated DOBC to calculate additional trajectory adjustments. By modifying the alpha value as convergence approached, we achieved a more stable path and quicker goal attainment. With minimal wind impact along the y-axis, we kept the beta gain low to reduce unnecessary adjustments. We also observed that the alpha value affected the initially set PID gains for x and attitude, prompting further manual tuning for the desired controller response.

## Conclusion

Our controller utilizes PID tuning for the y-axis, x-axis, and attitude, emphasizing speed and stability while accommodating different error conditions using the Dynamic PID. Integration of a DOBC enhances robustness, effectively countering external disturbances. The controller's blend of traditional and advanced control methods showcases a comprehensive approach to drone control.
