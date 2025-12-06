---
title: 'Ground Plane Detection with RANSAC'
published: 2025-11-09
draft: true
toc: true
tags: ['sensors', 'computer-vision']
description: 'An algorithm written for ARV 2025-2026. We used random sample consensus (RANSAC) to create a occupancy grid used for pathfinding around obstacles.'
author: ['the2nake', 'joshua kin', 'edison zhou']
---

This was my first completed project as part of the ARV club at UMich. Unlike the others, the direction for this project was more or less set by the higher-ups, so to speak. Our aims were to create an occupancy grid with sides of 5 cm which marked the driveable areas by the robot.

## constraints

We had access to the following resources:

* depth data calculated from the Zed stereo camera system
* corresponding camera intrinsics
* 30 ms of compute time on the onboard laptop

Having the depth data is really the crucial part, and I definitely appreciate not having to figure that out myself.

## first steps

We first outlined the RANSAC algorithm, which performs the following steps iteratively:

* sample 3 points and form a plane equation
* calculate an error heuristic for each point in the space
* tolerance that heuristic to get the metric (number of inlier points)
* if needed, update the best metric and corresponding plane equation

This algorithm suited our scenario for two reasons. It performs only two challenging operations: a matrix inverse for generating the plane equation, and calculation of the metric for each point.

### pooling

