---
title: 'Kalman Filter for Dead Wheel Odometry'
published: 2024-07-25
draft: false
toc: true
tags: ['controls', 'kalman-filter']
description: 'A project part of the 2024-25 VEX Robotics season. I created a Kalman filter to reduce errors in position tracking over time and improve motion repeatability.'
author: 'the2nake'
---

This project is what started my foray into some control theory beyond PID. Admittedly, Kalman filtering (even multidimensionally) is not a very complicated concept, and the implementation I made was neither elegant nor efficient. In personal terms, however, the completion of this project was a landmark moment. 

This was the first time I really understood linear algebra, and the first time I applied further mathematics to a robotic application.

## Context

Omnidirectional wheels (omniwheels) are used in odometry systems to track relative position over time. Odometry operates using a dead-reckoning approach: it adds robot-frame displacement vectors measured by the wheels to a tracked global displacement vector. An IMU provides the angle needed to perform the change of basis.

### Motivation

Previously, our team used 220mm travel omniwheels, which provided strong position tracking due to their even roundness and low friction laterally. We switched to smaller 160mm travel omniwheels to accommodate the reduced floorspace of a new overall design for the robot.

We found that the construction of the new wheels caused poorer tracking performance. The smaller wheels had only four roller axes, which caused friction problems when wheel rollers could be angled up to 45 degrees from the ground.

We prioritised the new physical design, which meant that we had to improve tracking performance in software. The Kalman filter was my answer to this problem, chosen because low-lag position tracking was vital for our motion algorithms and because of the many ways to tune it to best suit our application.

## The Kalman Filter

The equations are actually fairly simple, but the "intent" of the linear algebra was not immediately obvious to me. To spare you a recount of the many misconceptions and half-knowledges I formed along the way, here is how they work as I understand it now.