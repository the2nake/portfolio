---
title: 'FLIP Fluid Simulation'
published: 2025-11-12
draft: false
toc: true
tags: ['simulation']
description: 'FLuid Implicit Particle technique in C with SDL. This is underlying work for a clone of the fluid simulation pendant by mitxela.'
author: ['the2nake']
---

Writing simulations isn't one of my main interests, per se, but it is pretty useful for controls engineering. Also, I couldn't get my hands on a [pendant made by mitxela](https://mitxela.com/projects/fluid-pendant) in time, so I plan to make my own version at some point. Hence, I decided to do this FLIP (**FL**uid **I**mplicit **P**article) simulation, using [a video](https://www.youtube.com/watch?v=XmzBREkK8kY) by Ten Minute Physics as a reference.

I honestly had some trouble replicating the simulation behaviour. For no discernible reason, my simulation has a bit higher viscosity than expected, and some overcompression of particles at the bottom. Though I am not totally happy with the result, I haven't yet found how to fix it. I'll make an update once I do.

### demo

But first! a demo.

<div class="aspect-ratio-3_2">
<iframe class="video" src="https://www.youtube.com/embed/L2p2-VZA64A?si=P41QRPNOtwL6cuND&rel=0" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

### code

::github{repo="the2nake/flip"}

## algorithm summary

The FLIP algorithm simulates a visually-realistic fluid using a grid of cells populated with water particles. The particles carry the fluid between cells, while the grid cells are used to ensure that the fluid is incompressible. The cells work by balancing the inflow and outflow of each cell.

![flowchart of flip simulation](./flip_flowchart.jpg 'Figure 1: Flowchart of the simulation')

### approach

My main targets for this project included limiting memory usage (where possible) and making the code run as fast as possible. Both of these, I hope, will help in transferring the code to an embedded platform at some point.

[spoiler] I managed to hit 14ms per frame for >10k cells and >25k particles, so I think this turned out alright.

## implementation (+ challenges)

I tried to guide my approach with these ideas:

- Make each simulation step as divorced as possible from its preceding/following steps
- Try to limit calls to `malloc`, only using it for particles (particle count changes with different arrangements of cells)
- Use dual indices for cell data and single indices for other things

That last bit turned out to be unwise. The rationale, at the time, was that "this is nice and consistent" :clown_face:. However, this divide led to *many* errors when converting between flow rate indices and cell indices.

Another lesson learned. Mixing indexing schemes between coupled data is _not_ smart.

### particle collisions

With the walls, this is trivial, but I still _(sigh)_ managed to make some dumb mistakes. Notably, I trapped particles mid-air by pushing particles back by their velocity instead of the container wall normal.

There is one non-obvious point though: upon a collision, only the velocity component of the particle *orthogonal to the surface normal* should be kept.

Collisions between two particles, however, is more difficult. In a nutshell, I used a hash table of `cell -> particles in cell` to limit collision check distances. I'll probably describe this in more detail in another post.

### density computation

The idea is to smoothing a particle's density impact across cells. This is done using bilinear interpolation. Surprisingly, I got this right on the first try, aside from having the corner weights reversed.

Speaking of reversing corner weights, the output for that only looks slightly noisier than the correct output, so I think there is something to be said for knowing when not to chalk it up to "numerical noise" or some other hand-wavy explanation. On the other hand, sometimes that is the right answer.

```cpp title="density interpolation"

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

Enforcing incompressibility (a.k.a. projection) is done iteratively. The initial steps to achieve a fluid-like motion were pretty easy, but I got stuck here. If I had to guess, it's probably because there are a lot of numbers and it was a lot to keep in my head at the same time.

Some of my mistakes:

- Ignoring the boundaries attached to air cells
- Computing with `NAN` instead of `0`
- Swapping indices between flow rate fields `v1` and `v2`

Visualising the flow rates turned out to be most helpful for fixing these mistakes. _Especially_ for the last one. Here's a trimmed-down version of the code

```cpp title="projection step"
for (int n = 0; n < iters; ++n) {
  for (int i = 1; i < SIM_H - 1; ++i) {
    for (int j = 1; j < SIM_W - 1; ++j) {
      if (states[i][j] != water_e) continue;
      bool sl = states[i][j - 1] != solid_e;
      bool sr = states[i][j + 1] != solid_e;
      bool su = states[i - 1][j] != solid_e;
      bool sd = states[i + 1][j] != solid_e;
      int s = sl + sr + su + sd; // flow rates that can be balanced
      if (!s) continue;

      const int v1_i = i * (SIM_W + 1) + j;  // left flow rate (going in)
      const int v2_i = i * SIM_W + j;        // top flow rate (going in)
      float *vl = &v1[v1_i];
      float *vr = &v1[v1_i + 1];
      float *vu = &v2[v2_i];
      float *vd = &v2[v2_i + SIM_W];

      // higher net flow out will cause velocities to be adjusted inwards
      float flow = (*vr + *vd - *vl - *vu)                 // net flow
      flow -= k_stiffness * (densities[i][j] - DENSITY_0); // separate dense areas

      flow *= k_relax / s; // take the average
      *vl += sl * flow;
      *vr -= sr * flow;
      *vu += su * flow;
      *vd -= sd * flow;
    }
  }
}
```

I am still not fully finished dealing with issues here, I think. My simulation has a lot of viscosity, even before separating particles. Surely, the issue lies with incompressibility or with separation based on density. Or maybe something else. Anyway, something is up.

On the bright side, it looks good alright.

### particle to cell velocity

This step transfers particle velocities to cell velocities/flow rates using bilinear interpolation. For both the x and y flow rates, the weights are computed to the 4 enclosing flow rate locations, similar to how density is computed.

This one was really challenging, and I kept getting errors with the particles flying out of bounds or division by zero weight. I would often think that I had fixed them, and then they would come back affecting different particles instead.

Inspecting the problematic particles and sprinkling `assert` statements throughout the code to crash when something was up really helped here. However, like before, it is somehow not perfect. It fails to properly separate the particles at the bottom, and perhaps the bounciness that ensues breaks up the smooth vortices that should continue as a result of the incompressibility step.

## final results

## closing thoughts
