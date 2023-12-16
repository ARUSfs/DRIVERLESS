# Resampling in particle filters

Author: [Ana Novas](https://www.linkedin.com/in/ananovasgarcía)

Date: 15-12-2023

Translated and posted by [Jorge Muñoz](https://www.linkedin.com/in/jorge-mun-rod)

Original PDF (in Spanish): [Resampling en filtros de partículas](https://drive.google.com/file/d/1BeSw0TFBOTqozmyl_YfpbumdSd13tWDG/view?usp=sharing)

## Particles Filters
In particle filters algorithms:
1. We randomly generate {math}`N` particles. Our particles are vectors with 3 coordinates:
(x, y, yaw). Each one has a weight, {math}`w_i` , that indicates how likely it is that the particle
matches the real state of the car.
2. We predict the next state of the particles. We move the particles based on how
we predict the real system moves.
3. We update the weight of the particles based on a real sensor measurement. The particles
that are closer to matching the measurement have greater weight than those that do not match
so much with the data given by the sensor.
4. Resampling. It consists of discarding the improbable particles and replacing them
by copies of the most probable particles.
5. Calculate estimates. We can optionally compute a weighted average and
the covariance of the set of particles to obtain an estimate of the state.

At the beginning, we start with a set of particles with the same probability,
{math}`\frac{1}{N}` (for a set of {math}`N` particles), since we have no evidence
to give more weight to any in particular. There may only be a few
particles near the real position of the car. As the algorithm is executed,
the particles that do not match the measurements will have
less and less weight and only the particles that are close to the car
will have appreciable weights. We could have 5000 particles and only 3 have
non-negligible weights. This happens because the algorithm degenerates. This is
solved by resampling the particles.

To explain it intuitively, particle resampling algorithms
seek to discard particles with less weight (low probabilities) and replace them
by new particles with high probabilities. We can achieve this by duplicating the
particles with the highest weight and slightly dispersing them with the noise added in
the step of predicting the state of particles. This gives us a set of
particles in which the vast majority accurately represents the
probability distribution.

We do not resample at every theoretical time instant t, since for example
you are not receiving new information from the sensor, it will not benefit you
to do a resampling. To determine approximately when to resample,
we use what is called {math}`\textit{effective} N`, which approximately measures the number of
particles that contribute significantly to the probability distribution.
The formula is as follows:

```{math}
:label: eq:effectiveN
\hat{N}_{eff} = \frac{1}{\displaystyle\sum_{i=1}^{N} w_i^{2}} \hspace{1cm} \forall i \in \{1, \dots, N\}
```

If {math}`\hat{N}_{eff}` is less than a certain threshold, you have to resample. A good starting point is {math}`N/2`, but this may vary depending on the problem. It is also possible that {math}`\hat{N}_{eff} = N`, which would mean that the set of particles has converged to a single point (which is {math}`N` times in the set with weight equal to {math}`1/N`). If this happens very often, you would have to increase the number of particles or adjust the filter in some way.

### Multinomial resampling
It samples the current set of particles {math}`N` times (where the probability
to select any particle should be proportional to its weight), creating a new
set of particles from the sample.

The idea is simple. First, we calculate the cumulative sum of the weights:
element 1 of the cumulative sum would be the sum of elements 0 and 1 of the weights,
element 2 of the cumulative sum would be the sum of elements 0, 1 and 2 of the weights, etc.
This gives us a growing list of probabilities from 0 to 1. We can think of it as if the
interval {math}`[0,1]` was divided into subintervals, where the particles with the highest
weight have larger subintervals and therefore a greater probability of being chosen.

```{image} ../../_static/images/multinomial_resampling.png
:alt: Multinomial resampling
:align: center
```

To select a weight we generate a random number uniformly selected
between 0 and 1 and we use binary search to find its position in the array
of the cumulative sum.
In NumPy the command {math}`\texttt{searchsorted}` directly applies
the binary search algorithm.

```{image} ../../_static/images/multinomial_resampling_2.png
:alt: Multinomial resampling
:align: center
```

This algorithm has a time complexity of {math}`O(n \text{log}(n))`. There are algorithms
with {math}`O(n)`, but it is essential to know the {math}`\textit{multinomial resampling}`, since
the following algorithms can be understood as variations of this one.

### Residual Resampling
The {math}`\textit{residual resampling}` ensures us a uniform sampling on the set
of particles. We take the normalized weights and multiply them by $N$, and then
we use the integer part of each weight to define how many samples of that particle
are taken. For example, if the weight of a particle is {math}`0.0026` and we have {math}`N=1000`
particles, its new weight would be {math}`2.6`, so {math}`2` samples of that particle would be taken. This ensures us that the particles with high probabilities are taken at least once.

However, this does not generate {math}`N` particles. Therefore, we have to take into
account the {math}`\textbf{residues}`, the new weights minus their integer part (the
decimal part of the new weights) since we had ignored them. Then, we use
a simpler sampling like the {math}`\textit{multinomial}` to select the rest should
particles based on the residue. If we continue the example, the residual part
of our particle was {math}`2.6- \texttt{int}(2) = 0.6`. The residue is large, so the particle has a lot of probability of being selected again.

In the following example, we have used {math}`N=15`:

```{image} ../../_static/images/residual_resampling.png
:alt: Residual resampling
:align: center
```

### Stratified Resampling
This method seeks to make relatively uniform selections in the set of
particles. It works as follows: we divide the cumulative sum
in {math}`N` equal sections (strata) and then, within each
section, we choose a particle at random. This guarantees us that the
distance between two samples taken {math}`\in [0,\frac{2}{N}]`.

In the graph, we see the strata separated by blue vertical lines, and
in each one of them a random sample has been chosen.

````{image} ../../_static/images/stratified_resampling.png
:alt: Stratified resampling
:align: center
````
### Systematic Resampling
In this algorithm, as in the stratified one, the cumulative sum is divided into
{math}`N` sections (which are at a distance {math}`\frac{1}{N}`). Then, the idea is to choose
a random particle and take from the others
by a distance proportional to {math}`\frac{1}{N}` from it.

```{image} ../../_static/images/systematic_resampling.png
:alt: Systematic resampling
:align: center
```


