# Perception Node

This is the **Perception Node documentation** of ARUSfs.

```{warning}
This documentation is under development
```

## Camera basics

To represent how a camera, or even our eyes, work we use [Proyective Geomtetry](https://en.wikipedia.org/wiki/Projective_geometry). In a nutshell, we *project* all the points in a straight line, so that we may represent any point {math}`[x_1, x_2, x_3]` in {math}`\mathbb R^3` by the line it is contained by, represented as {math}`[x_1/x_3, x_2/x_3, 1]`. In other words, any point in the line {math}`[u, v, 1]` has coordinates {math}`s\cdot [u,v,1]`.

```{math}
\begin{bmatrix} 1&1\\1&1\end{bmatrix}
```
