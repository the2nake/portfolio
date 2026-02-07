---
title: 'Fusing depth vision for autonomous navigation'
published: 2026-02-06
draft: true
toc: true
tags: ['project']
description: 'Most of my work for U-M ARV during 2025-26. We used, among other techniques, RANSAC and point clouds to segment obstacles from the ground.'
author: 'the2nake, joshua kin, edison zhou, arnav'
---

This was my first project as part of the U-M ARV club. Unlike others that I've done, the direction for this project was more or less set by the "higher-ups", so to speak. Our aim was to create an occupancy grid with sides of 5 cm which marked the driveable areas by the robot.

### TODO - INSERT SOME VISUALS HERE

### code

::github{repo="umigv/UMARV-CV-ScenePerception"}

Check the `algorithms/algorithm_37bec7ux/src/ransac` directory for the code.

## objectives

This project was motivated by needing to determine obstacles without needing to categorise them. The grid output was chosen by the people doing navigation motion planning for its utility for pathfinding.

The project's final deliverable would be a ROS (Robot Operating System) node to publish a matrix that represented square areas on the ground as well as if they were occupied by some obstacle.

:::note
We kind of took our sweet time. Due to various bureaucratic factors, we only started planning on 25-10-19, and produced the first concepts around a week later.
:::

### constraints

Here's what we had to work with:

- 2x stereo camera system (Zed 2i)
  - RGB-D image (position, colour, and depth)
  - associated camera intrinsics
- 60 ms of compute time
- made in Python

Having the depth data is really the crucial part, and I appreciate not having to figure that out myself. However, that 60 ms compute time combined with the Python requirement is unenviable, especially when the task is loop-heavy.

C++ would've been my language of choice, but we used Python to conform with the rest of the project. By the time we encountered performance limitations, we had too much written to justify switching to C++ and getting the accompanying environment set up (at minimum, telemetry, CMake, SDL, Eigen, OpenCV, ROS). At that point, I didn't think it was worth the hassle, and now, I kind of regret making that call.

## first steps

We first set out to simply identify the areas of the ground on the image first, and get a sense of how the depth data might be used. This process is known as image segmentation.

### segmentation

We just need to figure out where we can move, so the segmented image is another image with white pixels where we can move and black pixels where we can't.

This kind of image, called a mask, lets us perform bitwise manipulations to combine it with information from, for example, HSV filtering, letting us produce different output for the white lane lines on the ground.

### algorithm choice

My favoured approach is to always prototype solutions before deciding on a final design, as seeing a design's affordances and limitations in the real world is almost always preferable to speculation and in-depth research.

In this vein, RANSAC was chosen for its ease of implementation, which would help us gauge algorithm complexity. In the end, we found it to be sufficiently robust for image segmentation.

:::note
There _is_ an optimal solution to plane fitting, given by $\vec{x^*}=R^{-1}Q^T\vec{b}$ (an equation I will not unpack, but it exists). However, this scales _spectacularly_ poorly when applied to the >300k points in a 720p image.
:::

### ransac

For the uninitiated, random sample consensus (RANSAC) creates a model to fit through a large number of data points by repeatedly picking a minimum number sufficient to fully constrain the model, then evaluating that model's "fit" through some metric like mean squared error (MSE).

We use it to detect the primary flat surface in the camera view (the ground plane). As a plane equation is described by $z(x,y)=ax+by+c$ with depth $z$ determined by position $(x,y)$, each RANSAC iteration will fit a plane equation to 3 random points in the image.

## pooling

Though iterative sampling is efficient, computing the error metric requires processing every single point in the image. This is undesirable, so we improved performance by downsampling the depth image (pooling).

By averaging a rectangle section of depth values into a single pixel's depth, we keep a coarse grained depth image with smaller size. The size of the section, called a "kernel", determines the speed up factor, as a larger kernel creates a smaller (and easier to process) pooled image.

### TODO - ADD GRAPHIC EXPLAINING POOLING KERNEL

:::tip
The choice of kernel shape is quite important. Larger kernels speed up the error metric computation during RANSAC, but they make the plane fit less accurate. To avoid sacrificing accuracy, we used a rectangular `1x10` pooling kernel.

This works because our cameras are mounted parallel to the horizon, which makes the depth data going horizontally across an image almost constant. With this pooling kernel, our metric calculation is 10x faster for no cost.
:::

Oftentimes, the best way to optimise is to do less :upside_down_face:.

## plane detection

To avoid converting every pixel into an equivalent point in 3-d space, we performed RANSAC on the image itself (pixel space). This requires modification of the equations for this different coordinate frame.

### pixel space vs. camera space

The camera space is the coordinate space in the real world in the reference frame of the camera. Though it is similar to the pixel space of the depth image, the x- and y-coordinates in camera space ($x_c$ and $y_c$) are dependent on the depth.

Imagine the line within the camera frustrum that corresponds to a pixel. Each pixel location could correspond to an infinite number of $(x_c, y_c)$ coordinates along said line. The depth value at a pixel's location determines which real coordinates are correct.

We use conversions derived from section 3 of [Focal Length and Intrinsic Camera Parameters | Baeldung on Computer Science](https://www.baeldung.com/cs/focal-length-intrinsic-camera-parameters). The relevant equations are shown below.

$$
\begin{align}
x_c&=z_c\frac{x_p-c_x}{f_x} \\
y_c&=z_c\frac{c_y-y_p}{f_y}
\end{align}
$$

Where variables subscripted with $c$ are in the camera space, $p$ are in the pixel space, and the rest are camera intrinsics.

:::important
These transformations are nonlinear, as depth $z$ varies with position $(x_p, y_p)$ in the image. They cannot be modelled either; the real coordinates of the obstacle pixels are also important.
:::

### plane equation in pixel space

The nonlinear relationship mentioned above also impacts how the pixel space plane is found. If we consider the general plane equation in camera space, depth is expressed as a linear function of $x$ and $y$.

$$
ax_c+by_c+c=z_c
$$

To work in pixel coordinates, substitution is required, resulting in the following plane equation. The second form is rearranged to separate coefficients that RANSAC would return.

$$
az_c\frac{x_p-c_x}{f_x}+bz_c\frac{c_y-y_p}{f_y}+c=z_c \\
-\frac{a}{cf_x}x_p+\frac{b}{cf_y}y_p+\left(\frac{ac_x}{cf_x}-\frac{bc_y}{cf_y}+\frac{1}{c}\right)=\frac{1}{z_c} \\
$$

I was surprised to see that the pixel space coordinates were linear to $\frac{1}{z_c}$, the inverse of depth. Full credit to Edison for catching this; I assumed the errors we got were a hardware limitation. Below you can see the naive fit with RANSAC (fitting to just depth).

![incorrect ransac plane fit](./arv_ransac_naive.jpg "Linear fit before considering space transforms")

In retrospect, that neat curve should've rung alarm bells. You can't catch everything alone, I suppose. And here is the plane fit algorithm after compensating correctly.

![correct ransac plane fit](./arv_ransac_corrected.jpg "Compensating for camera transforms gives correct model")
