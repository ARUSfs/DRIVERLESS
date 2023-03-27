# Perception Node

This is the **Perception Node documentation** of ARUSfs.

```{warning}
This documentation is under development
```

## Camera basics

To represent how a camera, or even our eyes, work we use [Affine Geometry](https://en.wikipedia.org/wiki/Affine_geometry) and [Projective Geomtetry](https://en.wikipedia.org/wiki/Projective_geometry). An affine space will be denoted by {math}`\mathbb A^n` and a projective one as {math}`\mathbb P^n`. In a nutshell, we *project* all the points in a straight line, so that we may represent any point {math}`[x_1, x_2, x_3]` in {math}`\mathbb A^3` by the line it is contained by, represented as {math}`[x_1/x_3, x_2/x_3, 1]`. In other words, any point in the line {math}`[u, v, 1]` has coordinates {math}`s\cdot [u,v,1]`.

With this in mind, we dive into the *pinhole camera* equation defined by:
```{math}
s\begin{bmatrix}u\\v\\1\end{bmatrix} = \begin{bmatrix}f_x&0&c_x\\0&f_y&c_y\\0&0&1\end{bmatrix}\begin{bmatrix}r_{11}&r_{12}&r_{13}&t_x\\r_{21}&r_{22}&r_{23}&t_y\\r_{31}&r_{32}&r_{33}&t_z\end{bmatrix}\begin{bmatrix}X\\Y\\Z\\1\end{bmatrix}
```

```{math}
\begin{bmatrix} 1&1\\1&1\end{bmatrix}
```
