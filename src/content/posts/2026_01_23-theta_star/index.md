---
title: 'Theta*: smoother pathfinding'
published: 2026-01-23
draft: false
toc: true
tags: ['algorithm']
description: 'An extension on the A* pathfinding algorithm that creates less complex paths by skipping certain nodes using a line-of-sight check with line rasterisation.'
author: 'the2nake'
coverImage:
  src: "./theta_star_demo.gif"
  alt: "An animation of the Theta* algorithm's output."
---

Another interesting algorithm, found this one in my ever-lengthening Watch Later playlist. Often used in robotics, Theta\* solves a problem with A\* that comes up when using a grid system: jagged paths when trying to move diagonally.

:::note
**pathfinding**: the process of finding the shortest path between two nodes in a graph (the mathematical structure). Like in many practical applications, our graph represents a physical space with obstacles as a grid whose squares (nodes) which can be traversable (ground) or not (obstacle). From any square, we can go its eight neighbours.
:::

Though Theta\* is meant to be used on top of A\*, which is more efficient in common physical environments (which have little variation in cost when moving between locations), the concept can also be identically applied to standard Dijkstra. My implementation of the algorithm can be found in the following repository:

::github{repo="the2nake/theta-star"}

## dijkstra

A bit of context into the algorithm Theta\* modifies: Dijkstra's algorithm. This algorithm explores a graph from the start node, incrementally exploring nodes which have the lowest total cost function until it explores the end node. Then, it works backwards to find the overall path, moving from the end node to the node that explored the end node, and so on until getting back to the start.

:::tip
The algorithm takes takes advantage of an abstract data type (the priority queue) that allows constant-time $O(1)$ access to the node with the least cost. This is usually done using a bit of magic called a min-heap.
:::

A\* (A-star) search is a Dijkstra variant which adds the value of a heuristic `h()` to the cost of a node. `h()` describes some metric which is lower for points more likely to lead to the end point. In practice, Euclidean or Manhattan distance are often used so that the algorithm explores points going closer to the goal first.

## line-of-sight smoothing

The idea of Theta\* is to remove unnecessary waypoints. A waypoint is unnecessary if there is an uninterrupted line of sight between its preceding and succeeding points; i.e. when it is possible to go straight and skip it.

This can be implemented in one of two ways: either generate the entire path using A\* and then "relax" the jagged path with the above idea, or perform line-of-sight checks when expanding new nodes.

The latter approach is better; the algorithm will be able to prioritise paths that have skipped points because they are straighter and have lower cost. It is not as fast, though, as it does more redundant checks.

Here's an example of naive post-processing failing, from [a paper on path smoothing](https://idm-lab.org/bib/abstracts/papers/socs20c.pdf) by Jihee Han and Tansel Uras.

![example where post-processing fails](./theta_star_post_process_example.png 'Figure 1: Post-processing an A* path is not optimal')

The post-processing approach generates the green dashed line. It fails to find the blue optimum solution because it cannot inject a new point, only remove points.

This skipping behaviour is shown below in Figure 2 for traversal across an open space, as well as a path around a corner where Theta\* cannot skip a point.

![diagram of the line of sight check](./theta_star_los_idea.jpg 'Figure 2: Theta* behaviour after line of sight check')

## implementation

Below is a code snippet from the repository linked in the introduction. I've labelled the areas of note and cut out some of the boilerplate so it is more obvious how the modification fits into the overall pathfinding algorithm. As you can see, Theta\* is not very complicated.

```cpp title="pathfinding with theta*" {"theta*: redirect if line of sight exists":25-30}
std::vector<coord> theta_star(grid& g, coord start, coord end) {
  min_queue<coord, float> frontier;
  std::vector<float> cost_sums(g.nodes.size(),
                               std::numeric_limits<float>::infinity());
  auto h = [&](coord c) { return dist(c, end); };
  auto cost_of = [&](coord from, coord to) {
    return cost_sums[g.as_idx(from)] + dist(from, to) * g[to].cost;
  };

  frontier.push({start, 0.f});
  cost_sums[g.as_idx(start)] = 0.f;

  while (!frontier.empty()) {
    coord curr = frontier.top().first;
    if (curr == end) break;
    frontier.pop();

    for (auto& next : g.neighbours(curr)) {
      if (g[next].is_wall()) continue;

      float new_cost = cost_of(curr, next);
      float& total_cost = cost_sums[g.as_idx(next)];
      if (new_cost >= total_cost) continue;  

      
      g[next].parent = curr;
      if (g.visible(g[curr].parent, next)) {
        g[next].parent = g[curr].parent;
        new_cost = cost_of(g[next].parent, next);
      }

      frontier.push({next, new_cost + h(next)});
      total_cost = new_cost;
    }
  }

  return reconstruct_from(end);
}
```

Of course, `g.visible()` has to be implemented. In the demo, I used [Bresenham's line algorithm](https://deepnight.net/tutorial/bresenham-magic-raycasting-line-of-sight-pathfinding/), a line rasterisation approach that uses only integer arithmetic.

The key idea is to express a line in standard form and divide it into half-planes ($ax+by+c \le 0$ would be the negative half-plane) that determine if the next pixel drawn should move only along the x-axis, or also move up at the same time. Notice that this approach will only handle lines with slope $\left|\frac{\Delta x}{\Delta y}\right| \le 1$.

The code below won't include that exact equation because of algebraic simplification. Another thing the snippet below does is handle orientation changes by defining a major (`M`) and minor (`m`) distance to draw the line, and a major step (`dM`) and minor step (`dm`) that is applied to shift along to the next pixel's coordinate.

```cpp title="bresenham line of sight"
bool grid::visible(coord a, coord b) {
  if (!in_bounds(a) || !in_bounds(b)) return false;

  coord curr = a;
  int dy = b.first - a.first;
  int dx = b.second - a.second;

  int M = std::abs(dx);
  int m = std::abs(dy);
  coord dM = {0, dx < 0 ? -1 : 1};
  coord dm = {dy < 0 ? -1 : 1, 0};

  if (m > M) {
    std::swap(dm, dM);
    std::swap(m, M);
  }

  for (int i = 0, j = 0; i <= M; ++i) {
    if (operator[](curr).is_wall()) return false;

    // within the negative half, the true line is above
    if (2 * (M * j - m * i - m) + M <= 0) {
      ++j;
      curr.first += dm.first;
      curr.second += dm.second;
    }
    curr.first += dM.first;
    curr.second += dM.second;
  }
  return true;
}
```

## animated demo

I'm currently working a nice way to "inject" visualisation steps into an algorithm to see how it is working (and maybe help debug it). Once that's figured out, I'll update this section with a GIF of that.

## closing thoughts

I think it's quite interesting to see how little of a change this is, yet how much the algorithm result differs. Namely, Theta\* turns out to not be optimal, even with an admissible heuristic (unlike A\*). This example, taken from the [video by Burton Ma](https://www.youtube.com/watch?v=vj1IhhDRNR4), shows one case where it fails.

![example where theta* fails](./theta_star_failure.png 'Figure 3: Theta* is not necessarily optimal')

Still, the result is very nearly optimal, within 0.2%. Here, the algorithm does not find the ideal path because it cannot switch parents to a visible point that is not a neighbour or the parent of a neighbour.

I suppose we can think of this as simply a refinement on A*, which itself is not optimal in open spaces anyway. It would stand to reason that this more complicated problem is not so easily resolved.
