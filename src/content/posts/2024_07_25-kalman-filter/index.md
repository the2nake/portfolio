---
title: 'Kalman Filter for Dead Wheel Odometry'
published: 2024-07-25
draft: false
toc: true
tags: ['sensors', 'controls', 'kalman-filter']
description: 'A project part of the 2024-25 VEX Robotics season. I created a Kalman filter to reduce errors in position tracking over time and improve motion repeatability.'
author: ['the2nake']
---

This project is what started my foray into some control theory beyond PID. Admittedly, Kalman filtering (even multidimensionally) is not a very complicated concept, and the implementation I made was neither elegant nor efficient. In personal terms, however, the completion of this project was a landmark moment.

This was the first time I really understood linear algebra, and the first time I applied further mathematics to a robotic application.

## Context

Omnidirectional wheels (omniwheels) are used in odometry systems to track relative position over time. Odometry operates using a dead-reckoning approach: it adds robot-frame displacement vectors measured by the wheels to a tracked global displacement vector. An IMU provides the angle needed to perform the change of basis.

### Motivation

Previously, our team used 220mm travel omniwheels, which provided strong position tracking due to their even roundness and low friction laterally. We switched to smaller 160mm travel omniwheels to accommodate the reduced floorspace of a new overall design for the robot.

We found that the construction of the new wheels caused poorer tracking performance. The smaller wheels had only four roller axes, which caused friction problems when wheel rollers could be angled up to 45 degrees from the ground.

We prioritised the new physical design, which meant that we had to improve tracking performance in software. The Kalman filter was my answer to this problem, chosen because low-lag position tracking was vital for our motion algorithms and because of the many ways to tune it to best suit our application.

## Equations

The equations are actually fairly simple, but the "intent" of the linear algebra was not immediately obvious to me. To spare you a recount of the many misconceptions and half-knowledges I formed along the way, here is how they work as I understand it now.

Notation is the most difficult part of understanding these equations.

| Expression        | Meaning                                              |
| ----------------- | ---------------------------------------------------- |
| $\bold{y}$         | true vector $\bold{y}$                               |
| $\hat{\bold{y}}$  | estimate vector $\bold{y}$                           |
| $\bold{T}$        | matrix $\bold{T}$                                    |
| $\bold{y}_{n\|m}$ | value of $\bold{y}$ at timestep $n$ computed at $m$. |

### predict

$$
\hat{\bold{x}}_{k|k-1} = \bold{F}\hat{\bold{x}}_{k-1|k-1} + \bold{G}\bold{u}_{k-1}
$$

This equation computes the estimated state of the system in the future (hence, the prediction step). Because it does not have measured information, it is also called _a priori_.

It consists of $\bold{F}\hat{\bold{x}}_{k-1|k-1}$, which models the how the system evolves over one timestep, added with $\bold{G}\bold{u}_{k-1}$, which models how known control inputs now (at $k-1$) will impact the state in the future at $k$.

$$
\bold{P}_{k|k-1} = \bold{F}\bold{P}_{k-1|k-1}\bold{F}^T + \bold{Q}
$$

This equation extrapolates the covariance of the state estimate computed above. $\bold{F}\bold{P}_{k-1|k-1}\bold{F}^T$ representes the how the current estimate's error spreads after one timestep.

It is possible to think of the effect of multiplying by $\bold{F}$ on the covariance of $\hat{\bold{x}}$ as analogous to how scalar multiplication has a quadratic effect on the variance. The precise derivation relies on finding $\bold{P}_{k|k}=E((\bold{e}_{k|k})(\bold{e}_{k|k})^T)$ where $\bold{e}_{k|k}=\bold{x}_{k|k}-\hat{\bold{x}}_{k|k}$, but this is not too interesting.

$\bold{Q}$ is the process noise covariance, that is, the error that is introduced through differences between the model equations and reality. For our usage, this was fairly high, because we weren't able to model the control input very well.

### correct

## Implementation

## Closing Thoughts

* At some point, I want to revisit modelling the control input. It would be useful to characterize the impact of a movement command on the position
* The extended or unscented variants of the filter are undoubtedly better suited for this application