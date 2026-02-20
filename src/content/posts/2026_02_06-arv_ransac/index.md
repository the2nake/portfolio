---
title: 'Depth vision for autonomous navigation (ARV)'
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

This project was motivated by needing to determine obstacles without needing to categorise them. The final deliverable would be a ROS (Robot Operating System) node to publish a matrix that represented square areas on the ground as well as if they were occupied by some obstacle.

:::note
We kind of took our sweet time. Due to various factors, we only started planning on 25-10-19, and produced the first concepts around a week later.
:::

### constraints

- 2x stereo camera system (Zed 2i)
  - RGB-D image (position, colour, and depth)
- 60 ms of compute time, made in Python

C++ would've been my language of choice, but we used Python to conform with the rest of the project. By the time we encountered performance limitations, we had too much written to justify switching to C++ and getting the accompanying environment set up (at minimum, telemetry, CMake, SDL, Eigen, OpenCV, ROS).

## goals

We first set out to simply identify the areas of the ground on the image first, and get a sense of how the depth data might be used. This process is known as image segmentation: we generate a black-and-white image which separates the areas of interest (lane lines and obstacles) and use it to locate obstacles. Here's the kind of result we wanted (and later achieved):

![example image segmentation mask](./arv_ransac_example_mask.png "Camera footage and generated driveable area mask (white)")

In the above picture, the white areas are the areas on the ground. We do highlight areas that are not ground at all (they cross the ground plane), but we post-process to ignore them in a later step.

## ransac

I like to just try stuff first and worry later. RANSAC (random sample consensus) was chosen because it's easy to implement, and we never saw need to change it. It's not optimal, but it's fast.

1. Repeatedly sample some points
2. Creating a model only through those points
3. Evaluating that model's "fit" for the whole dataset using mean squared error.

We use it to detect the primary flat surface in the camera view (the ground). A plane is described by $z(x,y)=ax+by+c$, so each RANSAC iteration samples three datapoints (pixel location with depth).

To avoid converting every pixel into an equivalent point in 3-d space, we performed RANSAC on the image itself (pixel space). I thought this was pretty slick, but the results weren't great at first:

![incorrect ransac plane fit](./arv_ransac_naive.jpg "Linear fit before considering space transforms")

The blue dots are the pixels the camera sees, arranged by depth. If you hadn't guessed, the ground is not supposed to curve like that. It's curved because we tried to shortcut by using pixel space, and the math changes.

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

Variables subscripted with $c$ are in the camera space, $p$ are in the pixel space, and the rest are camera intrinsics.

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
\left(-\frac{a}{cf_x}\right)x_p+\left(\frac{b}{cf_y}\right)y_p+\left(\frac{ac_x}{cf_x}-\frac{bc_y}{cf_y}+\frac{1}{c}\right)=\frac{1}{z_c} \\
$$

I was surprised to see that the pixel space coordinates were linear to $\frac{1}{z_c}$, the inverse of depth. Full credit to Edison for catching this; I had just assumed the errors we got were a hardware limitation.

In retrospect, I should've known that neat curve was not a coincidence. You can't catch everything alone, I suppose. And here is the plane fit algorithm after compensating correctly.

![correct ransac plane fit](./arv_ransac_corrected.jpg "Compensating for camera transforms gives correct model")

### pooling

Though iterative sampling is efficient, computing the error metric requires processing every single point in the image. This is undesirable, so we improved performance by downsampling the depth image (pooling).

By averaging a rectangle section of depth values into a single pixel's depth, we keep a coarse grained depth image with smaller size. The size of the section, called a "kernel", determines the speed up factor, as a larger kernel creates a smaller (and easier to process) pooled image.

:::tip
Having a large range of depth values being averaged will degrade RANSAC performance. We used a `1x16` kernel, which worked well because depth does not vary much along the x-axis.
:::

Oftentimes, the best way to optimise is to do less :upside_down_face:.
