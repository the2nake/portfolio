---
title: 'Obstacle segmentation with RANSAC'
published: 2025-12-13
draft: true
toc: true
tags: ['project', 'sensors']
description: 'An algorithm written for U-M ARV 2025-2026. We used random sample consensus (RANSAC) to segment obstacles from the ground.'
author: 'the2nake, joshua kin, edison zhou'
---

This was my first completed project as part of the U-M ARV club. Unlike others that I've done, the direction for this project was more or less set by the "higher-ups", so to speak. Our aims were to create an occupancy grid with sides of 5 cm which marked the driveable areas by the robot.

### TODO - INSERT SOME VISUALS HERE

### code

::github{repo="umigv/UMARV-CV-ScenePerception"}

Check the `algorithms/algorithm_37bec7ux/src/ransac` directory for the code.

## first steps

This project was motivated by needing to determine obstacles without needing to categorise them. The grid output was chosen by the people doing navigation motion planning for its utility for pathfinding.

The project's final deliverable would be a ROS (Robot Operating System) node to publish a matrix that represented square areas on the ground as well as if they were occupied by some obstacle.

:::note
We ... kind of took our sweet time. Due to various bureaucratic factors, we only started planning on 25-10-19, and produced the first concepts around a week later.
:::

### constraints

Here's what we had to work with:

- stereo camera system (Zed 2i)
  - RGB-D image (position, colour, and depth)
  - associated camera intrinsics
- 60 ms of compute time
- made in Python
- work within lab times (personal requirement)

Having the depth data is really the crucial part, and I appreciate not having to figure that out myself. However, that 60 ms compute time combined with the Python requirement is unenviable, especially when the task is loop-heavy.

C++ would've been my language of choice, but we used Python to be more accessible to the team. By the time we encountered performance limitations, we had too much written to justify switching to C++ and getting the accompanying environment set up (at minimum, CMake, SDL, Eigen, OpenCV, ROS). At that point, I didn't think it was worth the hassle, to be honest.

### algorithm choice

My personal approach is to always prototype possible solutions before deciding on a final design, as seeing a design's affordances and limitations in the real world is almost always preferable to speculation and in-depth research.

In this vein, RANSAC was our first pick to prototype, as its ease of implementation would help us gauge how complicated our algorithm would need to be. We eventually found it to be sufficiently robust, and saw no need to investigate other techniques.

:::note
There _is_ an optimal solution to plane fitting, given by $\vec{x^*}=R^{-1}Q^T\vec{b}$ (an equation I will not unpack, but it exists). However, this scales _spectacularly_ poorly when applied to the >300k points in a 720p image.
:::

### ransac

### TODO - INSERT RANSAC PLANE FIT TEST

For the uninitiated, random sample consensus (RANSAC) creates a model to fit through a large number of data points by repeatedly picking a minimum number sufficient to fully constrain the model, then evaluating that model's "fit" through some metric like mean squared error (MSE).

In our case, we use it to detect the primary flat surface in the camera view (the ground plane). As a plane equation is described by $z(x,y)=ax+by+c$ with depth $z$ determined by position $(x,y)$, each RANSAC iteration will fit a plane equation to 3 random points in the image.

## pooling

Though iterative sampling is quite efficient, calculating the error metric for the plane fit still requires processing every single point in the image. As this is undesirable, we can reduce the amount we process by scaling down the depth image (pooling).

There are many ways to do pooling, but we simply take the average of the depth data in a rectangular section and map it into a single pixel of a pooled image. This section, called a "kernel", determines the factor by which the error metric computation is sped up, as a larger kernel creates a smaller (and easier to process) pooled image.

### TODO - ADD GRAPHIC EXPLAINING POOLING KERNEL

:::tip
The choice of kernel shape is quite important. Larger kernels speed up the error metric computation during RANSAC, but they make the plane fit less accurate. To avoid sacrificing accuracy, we used a rectangular `1x10` pooling kernel.

This works because our cameras are mounted parallel to the horizon, which makes the depth data going horizontally across an image almost constant. With this pooling kernel, our metric calculation is 10x faster for no cost.
:::

Oftentimes, the best way to optimise is to do less :upside_down_face:.

## plane detection

### pixel space vs. camera space

First, it is important to determine how any point on the depth image (pixel space) corresponds to a point in the real world (camera space).

The camera space is the coordinate space in the real world in the reference frame of the camera. Though it is similar to the pixel space of the depth image, the x- and y-coordinates in camera space ($x_c$ and $y_c$) are dependent on the depth.

This distinction can be visualised by imagining the line within the camera frustrum that corresponds to a pixel. As photography is a mapping of 3-d space onto a 2-d image, each pixel location could correspond to an infinite number of $(x_c, y_c)$ coordinates. In a specific depth image, the depth value at a pixel's location would determine which real coordinates are used.

We convert the coordinates using formulae explained in section 3 of [Focal Length and Intrinsic Camera Parameters | Baeldung on Computer Science](https://www.baeldung.com/cs/focal-length-intrinsic-camera-parameters). The relevant equations are shown below.

$$
\begin{align}
x_p&=f_x\frac{x_c}{z}+c_x \\
y_p&=f_y\frac{y_c}{z}+c_y
\end{align}
$$

:::important
These transformations are nonlinear, as depth $z$ varies with position $(x_p, y_p)$ in the image. They cannot be modelled either; as will become apparent later, the real coordinates of the obstacle pixels are also important.
:::

### plane equation in pixel space

The nonlinear relationship mentioned above also impacts how the pixel space plane is found. If we consider the general plane equation in camera space (which is just a rotation of real world coordinates), the

talk about mistake made when assuming linear fit from ransac.
