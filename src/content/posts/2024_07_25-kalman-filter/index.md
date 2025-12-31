---
title: 'Kalman Filter for Dead Wheel Odometry'
published: 2024-07-25
draft: true
toc: true
tags: ['project', 'sensors', 'controls']
description: 'A project part of the 2024-25 VEX Robotics season. I created a Kalman filter to reduce errors in position tracking over time and improve motion repeatability.'
author: 'the2nake'
---

## [TODO] make this a short post and do the project in further detail in the current time (writing docs without coding is dumb and you're simulating the past anyway)

This project is what started my foray into some control theory beyond PID. Admittedly, Kalman filtering (even multi-dimensionally) is not a very complicated concept, and the implementation I made was neither elegant nor efficient. In personal terms, however, the completion of this project was a landmark moment.

This was the first time I really understood linear algebra, and the first time I applied further mathematics to a robotic application.

## context

We track robot position with omnidirectional wheels (omniwheels) and odometry. Odometry operates using a dead-reckoning approach: it adds robot-frame displacement vectors measured by the wheels to a tracked global displacement vector. An IMU provides the angle needed to perform the change of basis.

Previously, our team used 220mm travel omniwheels, which provided strong position tracking due to their even roundness and low friction laterally. We switched to smaller 160mm travel omniwheels to accommodate the reduced space in a new overall design.

We found that the construction of the new wheels caused poorer tracking performance. The smaller wheels had only four roller axes, which caused friction problems when wheel rollers could be angled up to 45 degrees from the ground.

We prioritised the new physical design, which meant that we had to improve tracking performance in software. The Kalman filter was my answer to this problem, chosen because low-lag position tracking was vital for our motion algorithms and because of the many ways to tune it to best suit our application.

## equations

The equations are actually fairly simple, but the "intent" of the linear algebra was not immediately obvious to me. Here's how I would summarise them after completing this project.

Note: notation _by far_ is the most difficult part of understanding these equations.

| Expression        | Meaning                                              |
| ----------------- | ---------------------------------------------------- |
| $\bold{y}$        | true vector $\bold{y}$                               |
| $\hat{\bold{y}}$  | estimate vector $\bold{y}$                           |
| $\bold{T}$        | matrix $\bold{T}$                                    |
| $\bold{y}_{n\|m}$ | value of $\bold{y}$ at timestep $n$ computed at $m$. |

### state space representation

We used a state vector $\bold{x} = \begin{bmatrix} x & \dot{x} & \ddot{x} & y & \dot{y} & \ddot{y} & \theta & \dot{\theta} \end{bmatrix}^T$. The entries represent the position and derivatives, as well as heading and angular velocity.

### predict

$$
\begin{equation}
\hat{\bold{x}}_{k|k-1} = \bold{F}\hat{\bold{x}}_{k-1|k-1} + \bold{G}\bold{u}_{k-1}
\end{equation}
$$

Computes the estimated state of the system in the future. $\bold{F}\hat{\bold{x}}_{k-1|k-1}$ models the how the system evolves over one timestep, and $\bold{G}\bold{u}_{k-1}$ models how known control inputs now (at time $n=k-1$) will impact the state in the future at $k$.

$$
\bold{F}=\begin{bmatrix} \
1 & \Delta{t} & \frac{\Delta{t}^2}{2} &   &           &                       &   &           \\
0 &        1  &       \Delta{t}       &   &           &                       &   &           \\
0 &        0  &              1        &   &           &                       &   &           \\
  &           &                       & 1 & \Delta{t} & \frac{\Delta{t}^2}{2} &   &           \\
  &           &                       & 0 &        1  &       \Delta{t}       &   &           \\
  &           &                       & 0 &        0  &              1        &   &           \\
  &           &                       &   &           &                       & 1 & \Delta{t} \\
  &           &                       &   &           &                       & 0 &        1  \\
\end{bmatrix}
$$

We derived the above state transition model from kinematic equations. We chose not to use a control input, so $\bold{u}_{k}=\vec{0}$.

$$
\begin{equation}
\bold{P}_{k|k-1} = \bold{F}\bold{P}_{k-1|k-1}\bold{F}^T + \bold{Q}
\end{equation}
$$

Extrapolates the covariance of the state estimate computed above. $\bold{F}\bold{P}_{k-1|k-1}\bold{F}^T$ is the expansion of the current estimate's error after one timestep.

Multiplying by $\bold{F}$ on the covariance of $\hat{\bold{x}}$ is kind of analogous to how scalar multiplication has a quadratic effect on the variance. The actual expansion, based on $\bold{P}_{k|k}=E((\bold{e}_{k|k})(\bold{e}_{k|k})^T)$ where $\bold{e}_{k|k}=\bold{x}_{k|k}-\hat{\bold{x}}_{k|k}$, is not too insightful.

$\bold{Q}$ is error that is introduced through incorrect modelling. We computed this by passing a tunable noise through our transition model. In hindsight, a control input from intended maneuvers would've been perfect here.

Here it is, where $\bold{F}_x$, $\bold{F}_y$, and $\bold{F}_\theta$ correspond to the blocks in matrix $\bold{F}_\theta$.

$$
\bold{Q} = \\
\begin{bmatrix} \\
\bold{F}_x\begin{bmatrix} 0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & 0 & \sigma^2_{\ddot x}\end{bmatrix}\bold{F}^T_x & & \\
& \bold{F}_y\begin{bmatrix} 0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & 0 & \sigma^2_{\ddot y}\end{bmatrix}\bold{F}^T_y  & \\
& & \bold{F}_\theta\begin{bmatrix}0 & 0 \\ 0 & \sigma^2_{\dot \theta}\end{bmatrix}\bold{F}^T_\theta\end{bmatrix}
$$

### correct

## implementation

## closing thoughts

- At some point, I want to revisit modelling the control input. It would be useful to characterize the impact of a movement command on the position
- The extended or unscented variants of the Kalman filter are indubitably better suited for this application
