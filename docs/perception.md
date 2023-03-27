# Perception Node

```{warning}
This documentation is under development
```

## Camera basics

To represent how a camera, or even our eyes, work we use [Affine Geometry](https://en.wikipedia.org/wiki/Affine_geometry) and [Projective Geometry](https://en.wikipedia.org/wiki/Projective_geometry). An affine space will be denoted by {math}`\mathbb A^n` and a projective one as {math}`\mathbb P^n`. In a nutshell, we *project* all the points in a straight line, so that we may represent any point {math}`[x_1, x_2, x_3]` in {math}`\mathbb R^3` by the line it is contained by, represented as {math}`[x_1/x_3, x_2/x_3, 1]`. In other words, any point in the line {math}`[u, v, 1]` has coordinates {math}`s\cdot [u,v,1]`.

With this in mind, we dive into the *pinhole camera*[^pinhole] equation defined by:
```{math}
:label: pinhole
s\begin{bmatrix}u\\v\\1\end{bmatrix} = \begin{bmatrix}f_x&0&c_x\\0&f_y&c_y\\0&0&1\end{bmatrix}\begin{bmatrix}r_{11}&r_{12}&r_{13}&t_x\\r_{21}&r_{22}&r_{23}&t_y\\r_{31}&r_{32}&r_{33}&t_z\end{bmatrix}\begin{bmatrix}X\\Y\\Z\\1\end{bmatrix}
```
From right to left, we have a point in {math}`\mathbb A^3`, a reference frame change describing the translation and rotation of the world reference frame {math}`\mathcal R_w` to the camera reference frame {math}`\mathcal R_c`, as well as the projection from {math}`\mathbb A^3` to {math}`\mathbb P^2` (also called extrinsics matrix); the camera matrix that models the focal length and center of the camera (also called intrinsics matrix), and finally {math}`u` and {math}`v` representing the pixels where the 3d point will appear on.

```{figure} https://docs.opencv.org/4.x/pinhole_camera_model.png
:scale: 1%
:alt: Pinhole camera model
Visualization of how {eq}`pinhole` works.
```

Seems simple enough but mind you, this is a simplified model for **pinhole cameras**, not real cameras. The difference comes because of camera lenses, which distort images. The best known distortion is the *fish lens* efect that GoPros or 360º cameras experience; though there are more types of distortion, described in the OpenCV Documentation[^pinhole]. In theory it isn't too much of an issue, but it is certainly an inconvenience. Instead of only having the linear nature of {eq}`pinhole`, we have to apply some *relatively complicated* non-linear undistortions which will slow down our pipeline and introduce numerican instabilities.

```{figure} https://docs.opencv.org/4.x/distortion_examples.png
:scale: 70%
:alt: distortion examples
Some examples of radial distortion on a chessboard pattern.
```

[^pinhole]: See the *Detailed Description* section of the [OpenCV documentation](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html) for more information.
