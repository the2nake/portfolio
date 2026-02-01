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

Another interesting algorithm, found this one in my ever-lengthening Watch Later playlist. It solves a problem with A* that comes up when generating a path for a physical robot using a grid system: jagged paths when trying to move diagonally.

:::note
**pathfinding**: the process of finding the shortest path between two nodes in a graph (the mathematical structure). Like in many practical applications, our graph represents a physical space with obstacles as a grid whose squares (nodes) which can be traversable (ground) or not (obstacle). From any square, we can go its eight neighbours.
:::

I'll first cover a short summary of Dijkstra's algorithm and A*, the basic pathfinding algorithms, before getting to the pseudocode of Theta* and how its modifications are structured. Though it is meant to be used on top of A*, which is more suitable for physical movement (which has no variation in cost when moving between locations), the concept can also be identically applied to standard Dijkstra.

::github{repo="the2nake/theta-star"}

## dijkstra

Dijkstra's algorithm is the breadth-first search of weighted graphs, guaranteed to find the shortest path. It does this through repeating the process of "expanding" nodes, consisting of:

1. for each non-wall neighbour of the expanded node, compute `maybe_cost = node.total + neighbour.cost`
2. if the neighbour hasn't been queued or`maybe_cost < neighbour.total`
   - update `neighbour.total = maybe_cost`
   - update `neighbour.pred = node`
   - add the neighbour to the queue with priority `neighbour.total`
3. expand the node with the next highest priority in the queue

We start by expanding the starting node, which has `node.total = 0`. By following this process, eventually we will expand the end node or have no more nodes to expand. The latter case indicates that no path exists.

In the former case, we can generate the full path by walking backwards from the end node. By that I mean going to the `.pred` of the end node, which has its own `.pred`, and so on until `.pred` is the starting node.

:::tip
The algorithm takes takes advantage of an abstract data type (the priority queue) that allows constant-time $O(1)$ access to the node with the least cost. This is usually done using a bit of magic called a min-heap.
:::

## theta* approach

The algorithm checks for a line of sight between waypoints, and skip intermediary points between them. Instead, the path will flow to the farthest point along the path within sight. Though it is entirely possible to edit paths afterward to give them this quality, modifying the algorithm to account for the new cost function will likely produce better results; the algorithm will prioritise movements that have skipped points because they are straighter and have lower cost.

Post still under construction, lol. Just uploading the results for now.
