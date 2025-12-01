---
title: 'FLIP Fluid Simulation'
published: 2025-11-12
draft: true
toc: true
tags: ['simulation']
description: 'FLuid Implicit Particle technique in C with SDL. This is underlying work for a clone of the fluid simulation pendant by mitxela.'
author: ['the2nake']
---

Writing simulations isn't one of my main interests, per se, but it is pretty useful for controls engineering. Also, I couldn't get my hands on a [pendant made by mitxela](https://mitxela.com/projects/fluid-pendant) in time, so I plan to make my own version at some point. Hence, I decided to do this FLIP (**FL**uid **I**mplicit **P**article) simulation, using [a video](https://www.youtube.com/watch?v=XmzBREkK8kY) by Ten Minute Physics as a reference.

I honestly had a lot of trouble replicating the simulation behaviour. I think I understand what every part does, and hopefully can explain it here, but there must be some detail in the implementation that I am missing. My simulation has issues with high viscosity, and the fluid particles are compressed at the bottom.

Though I am not really happy with the result, I haven't yet found how to fix it. I will try a direct port of the code at some point.

## algorithm summary

The FLIP algorithm simulates a visually-realistic fluid using a grid of cells populated with water particles. The particles carry the water between cells, while the grid cells are used to ensure that the fluid is incompressible (i.e. inflow and outflow of each cell throughout the fluid are balanced).

[ insert a flowchart here lol ]

In broad strokes, the steps for this algorithm are as follows:

- fill the grid cells with starting `solid`, `air`, or `fluid` states
- distribute fluid particles in `fluid` cells

And loop the following:

- pull particles using gravity
- solve particle collisions
- compute densities in each cell
- transfer particle velocities to cells
- enforce incompressibility
- transfer cell flow velocities to particles

### approach

My main targets for this project included limiting memory usage (where possible) and making the code run as fast as possible. Both of these, I hope, will help in transferring the code to an embedded platform at some point.

[spoiler] I managed to hit 14ms per frame for >10k cells and >25k particles, so I think this turned out alright.

## implementation (+ challenges)

Deciding how to organise the data for the cells and particles was tricky. I tried to statically allocate as much memory as possible, and dynamically allocated only the particle-related arrays to save memory. I am happy with how that turned out, but I should've paid more attention to clarity.

I tried to keep a consistent pattern of dual indices for accessing cell data (row and column) and single indices for everything else (flow rates, particles, weights). The rationale at the time was "this is nice and consistent". However, this divide led to _many_ errors when converting between indices of flow rates and indices of cells.

In retrospect, mixing indexing schemes between coupled data was _not_ smart.

### particle collisions

With the walls, this is trivial, but I still _(sigh)_ managed to make some dumb mistakes. Notably, I trapped particles mid-air by pushing particles back by their velocity instead of the container wall normal.

Collisions between two particles, however, is more difficult. In a nutshell, I used a hash table of `cell -> particles in cell` to limit collision check distances. I'll probably describe this in more detail in another post.

### density computation

The idea is to smoothing a particle's density impact across cells. This is done using bilinear interpolation.

```cpp
/*--- compute density ---*/

// c00--|--c01     grid centered on the particle
//  | . | . |      cxx indicates center of the cell
//  |---|---|      areas marked with a `.` are the interpolation
//  | . | . |        factors for the *opposite* corner
// c10--|--c11

// find indices for the 2x2 cells containing the particle
int i0 = particles[i].x2 / CELL_H - 0.5f;
int j0 = particles[i].x1 / CELL_W - 0.5f;
int i1 = imin(i0 + 1, SIM_H - 1);
int j1 = imin(j0 + 1, SIM_W - 1);

// interpolation factor for the area closer to the bottom-right
float f1_x1 = particles[i].x1 / CELL_W - (j0 + 0.5f);
float f1_x2 = particles[i].x2 / CELL_H - (i0 + 0.5f);
// for area closer to the top-left
float f0_x1 = 1.f - f1_x1;
float f0_x2 = 1.f - f1_x2;

float *surrounding[4] = {&densities[i0][j0], &densities[i0][j1],
                         &densities[i1][j0], &densities[i1][j1]};
float factors[4] = {f0_x2 * f0_x1, f0_x2 * f1_x1,  //
                    f1_x2 * f0_x1, f1_x2 * f1_x1};

// redistribute density if hits a solid
//   the ones that are solid have density of `nan` already
float sum_density = 0.f;
for (int k = 0; k < 4; ++k) {
  sum_density += isnan(*surrounding[k]) ? 0.f : factors[k];
}
if (sum_density > 0) {
  for (int k = 0; k < 4; ++k) {
    *surrounding[k] += factors[k] / sum_density;
  }
}
```

### incompressibility

The initial steps to achieve a fluid-like motion were pretty easy, but I first was stuck on solving for incompressibility.

I had first made a mistake where I ignored the boundaries attached to air cells.

Visualising the flow rates turned out to be most helpful for fixing these mistakes.

I am still not fully finished dealing with issues here, I think. My simulation has a lot of viscosity, even before separating particles. Surely, the issue lies with incompressibility or with separation based on density. Either way, the projection step must be doing something wrong.

### particle to cell velocity

This step transfers particle velocities to cell velocities/flow rates using bilinear interpolation. For both the x and y flow rates, the weights are computed to the 4 enclosing flow rate locations, similar to how density is computed.

This one was really challenging, and I kept getting errors with the particles flying out of bounds or division by zero weight. I would often think that I had fixed them, and then they would come back affecting different particles instead.

Inspecting the problematic particles and sprinkling `assert` statements throughout the code to crash when something was up really helped here. However, like before, it is somehow not perfect. It fails to properly separate the particles at the bottom, and perhaps the bounciness that ensues breaks up the smooth vortices that should continue as a result of the incompressibility step.

## final results

## closing thoughts
