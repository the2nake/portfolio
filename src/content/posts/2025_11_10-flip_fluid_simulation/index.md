---
title: 'FLIP Fluid Simulation'
published: 2025-11-12
draft: true
toc: true
tags: ['simulation', 'fluid']
description: 'FLIP implementation in C with SDL. This is some underlying work for a clone of the fluid simulation pendant by mitxela.'
author: ['the2nake']
---

Writing simulations isn't one of my main interests, per se, but it is pretty useful for controls engineering. Also, I couldn't get my hands on a [pendant made by mitxela](https://mitxela.com/projects/fluid-pendant) in time, so I plan to make my own version at some point. Hence, I decided to do this FLIP (**FL**uid **I**mplicit **P**article) simulation, using [a video](https://www.youtube.com/watch?v=XmzBREkK8kY) by Ten Minute Physics as a reference.

## First Steps

We first outlined the RANSAC algorithm, which performs the following steps iteratively:

- sample 3 points and form a plane equation
- calculate an error heuristic for each point in the space
- tolerance that heuristic to get the metric (number of inlier points)
- if needed, update the best metric and corresponding plane equation

This algorithm suited our scenario for two reasons. It performs only two challenging operations: a matrix inverse for generating the plane equation, and calculation of the metric for each point.

### Pooling
