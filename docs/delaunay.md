# Delaunay Detector Node

We define planning as the process of taking a given map with cones, and from it extracting tracklimits and the desired path for our car to take. With respect to path-planning, we are currently only concerned with extracting the middle path of a track. After performing some tests with our Laptime Simulator, we observed that the time difference between the middle path and the *time-optimized* path wasn't significant enough to justify it's more complex implementation and slower execution time. There are are middle points, such as minimum curvature, which may be useful in some contexts.

## Delaunay triangulation
For both track-limits and middle-path extraction we make use of Delaunay triangulation[^delaunay]. We may define a Delaunay triangulation of a given set of points **P** as

> A triangulation of **P** which for any given triangle, there is no point {math}`p\in P` strictly contained within the circumcircle of the triangle.

[Delaunay trianguluation wikipedia page](https://en.wikipedia.org/wiki/Delaunay_triangulation)
