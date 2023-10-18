# LiDAR Perception Node

```{warning}
This documentation is under development
```

## LiDAR basics
The [LiDAR](https://en.wikipedia.org/wiki/Lidar) sensor returns a 360º pointcloud by emitting thousands of infrared light beams per second, which is transformed into an array of points (x,y,z,i), where "x,y,z" are the coordinates of the points detected (being the LiDAR the origin) and "i" is the instensity of return of the beam.

```{figure} https://i.ibb.co/jMw5CYh/pcloud.png
:width: 600 px
Raw point cloud.
```

First of all, we can discard those points that are too far away, behind the car, or too high, just with a simple filter

Next, we need to remove the points corresponding to the floor, since they won't be useful. This can be done by fitting the equation of a plane using [RANSAC](https://es.wikipedia.org/wiki/RANSAC) and removing all points near the plane, or by dividing the 3d space in multiple segments (by angle and distance) and removing the points at the lowest height of each segment.

```{figure} https://i.ibb.co/kmpKJ1X/ground-rm-pcloud.png
:width: 600 px
Ground removed point cloud.
```

After ground removal, we are able to group the remaining points in the cloud by using Euclidean Clustering, which groups the points into different objects by the density distribution of the points using the euclidean distance. 


Once all the points are grouped in clusters, we can calculate a score for each one representing how similar the cluster is to a real cone (he clusters with a high enough score are considered as real cones) 
```{math}
:label: score
score = \frac{1}{n}\sum_{i=1}^{n} 1 - min\left ( \frac{d{_{i}}^{2}}{t^{2}},1 \right )

```

```{figure} https://i.ibb.co/9w5JKwT/clusters-scoring.png
:width: 400 px
Cluster scoring.
```


Although the LiDAR cannot detect colors directly, we can estimate wether a cone is blue or yellow because of the strip in it's center. This strip is white on blue cones and black on yellow cones, so we can differenciate them because blue cones will have a higher intensity at the center, and yellow cones the opposite. 

```{figure} https://i.ibb.co/X4TtmhZ/color-estimation.png
:width: 700 px
Cone color estimation.
```