# Perception Node

This is the **Perception Node documentation** of ARUSfs.

```{warning}
This documentation is under development
```

## Camera basics

To represent how a camera, or even our eyes, work we use [Proyective Geomtetry](https://en.wikipedia.org/wiki/Projective_geometry). In a nutshell, we *project* all the points in a straight line, so that we may represent any point `[x_1, x_2, x_3]`{math} in `\mathbb R^3`{math} by the line it is contained by, represented as `[x_1/x_3, x_2/x_3, 1]`{math}. In other words, any point in the line `[u, v, 1]`{math} has coordinates `s\cdot [u,v,1]`{math}.

```{math}
\begin{bmatrix} 1&1\\1&1\end{bmatrix}
```
