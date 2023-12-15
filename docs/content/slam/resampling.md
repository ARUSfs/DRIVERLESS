# Resampling in particle filters

Author: [Ana Novas](https://www.linkedin.com/in/ananovasgarcía)

Date: 15-12-2023

Translated and posted by [Jorge Muñoz](https://www.linkedin.com/in/jorge-mun-rod)

Original PDF (in Spanish): [Resampling en filtros de partículas]()

## Filtros de partículas
En los algoritmos de filtros de partículas:
1. Generamos aleatoriamente {math}`N` partículas. Nuestras partı́culas serán vectores con 3 coordenadas:
(x, y, yaw). Cada una tiene un peso, {math}`w_i` , que indica cómo de probable es que coincida la partı́cula con
el estado real del coche.
2. Predecimos el siguiente estado de las partı́culas. Movemos las partı́culas basándonos en cómo
predecimos que el sistema real se mueve.
3. Actualizamos el peso de las partı́culas basándonos en una medición real del sensor. Las partı́culas
que están más cerca de coincidir con la medición tienen mayor peso que las que no coinciden tanto con
el dato dado por el sensor.
4. Resampling. Consiste en descartar las partículas improbables y remplazarlas
por copias de las partículas más probables.  
5. Calcular estimaciones. Opcionalmente podemos computar una media ponderada y
la covarianza del set de partículas para obtener una estimación del estado.  

Al principio, comenzamos con un conjunto de partículas con la misma probabilidad,
{math}`\frac{1}{N}` (para un conjunto de {math}`N` partículas), pues no tenemos evidencias 
para darle más peso a ninguna en especial. Puede que solo haya unas pocas 
partículas cerca de la posición real del coche. A medida que se va ejecutando 
el algoritmo, las partículas que no coinciden con las mediciones irán teniendo 
cada vez un peso menor y tan solo las partículas que estén cerca del coche 
tendrán pesos apreciables. Podríamos tener 5000 partículas y que solo 3 tengan 
pesos no despreciables. Esto ocurre porque el algoritmo se degenera. Esto se 
resuelve resampleando las partículas.  

Para explicarlo de forma intuitiva, los algoritmos de resampleo de partículas 
buscan descartar las partículas con menos pesos (probabilidades bajas) y reemplazarlas
por nuevas partículas con probabilidades altas. Podemos conseguirlo duplicando las 
partículas con mayor peso y dispersándolas ligeramente con el ruido añadido en 
el paso de predicción del estado de partículas. Esto nos da un conjunto de 
partículas en el que la gran mayoría representa de forma precisa la distribución
de probabilidad. 

No hacemos resampleo en cada instante de tiempo teórico t, ya que si por ejemplo
no estás recibiendo información nueva del sensor, no va a beneficiarte en nada 
hacer un resampling. Para determinar aproximadamente cuándo hacer un resampleo,
usamos lo que se llama \textit{effective} $N$, que mide aproximadamente el número de 
partículas que contribuyen significativamente a la distribución de probabilidad.
La fórmula es la siguiente:  

```{math}
:label: eq:effectiveN
\hat{N}_{eff} = \frac{1}{\displaystyle\sum_{i=1}^{N} w_i^{2}} \hspace{1cm} \forall i \in \{1, \dots, N\}
```

Si {math}`\hat{N}_{eff}` es menor a cierto umbral, hay que resamplear. Un buen punto
de partida es {math}`N/2`, pero esto puede variar según el problema. Es posible también
que {math}`\hat{N}_{eff} = N`, que significaría que el conjunto de partículas ha
covergido a un solo punto (que está {math}`N` veces en el conjunto con peso igual a {math}`1/N`). 
Si esto ocurre muy a menudo, habría que incrementar el número de partículas o ajustar
el filtro de alguna manera. 

### Multinomial resampling
Hace un muestreo del conjunto actual de partículas {math}`N` veces (donde la probabilidad
de seleccionar cualquier partícula debería ser proporcional a su peso), creando un nuevo
conjunto de partículas a partir de la muestra.

La idea es sencilla. Primero, calculamos la suma acumulada de los pesos: 
el elemento 1 de la suma acumulada sería la suma de los elementos 0 y 1 de los pesos,
el elemento 2 de la suma acumulada sería la suma de los elementos 0, 1 y 2 de los pesos, etc.
Esto nos da una lista creciente de probabilidades de 0 a 1. Podemos pensarlo como si el 
intervalo {math}`[0,1]` estuviera dividido en subintervalos, donde las partículas con mayor
peso tienen subintervalos más grandes y por tanto, mayor probabilidad de ser escogidas.

```{image} ../../_static/images/multinomial_resampling.png
:alt: Multinomial resampling
:align: center
```

Para seleccionar un peso generamos un número aleatorio uniformemente seleccionado
entre 0 y 1 y usamos la búsqueda binaria para encontrar su posición en el array
de la suma acumulada.  
En NumPy el comando {math}`\texttt{searchsorted}` aplica directamente
el algoritmo de búsqueda binaria. 

```{image} ../../_static/images/multinomial_resampling_2.png
:alt: Multinomial resampling
:align: center
```

Este algoritmo tiene una complejidad temporal de {math}`O(n \text{log}(n))`. Hay algoritmos
con {math}`O(n)`, pero es esencial conocer el {math}`\textit{multinomial resampling}`, ya que 
los siguientes algoritmos pueden entenderse como variaciones de este. 

### Residual Resampling
El {math}`\textit{residual resampling}` nos asegura un muestreo uniforme sobre el conjunto
de partículas. Se toman los pesos normalizados y se multiplican por $N$, y luego 
se usa la parte entera de cada peso para definir cuántas muestras de esa partícula
se toman. Por ejemplo, si el peso de una partícula es {math}`0.0026` y tenemos {math}`N=1000`
partículas, su nuevo peso sería {math}`2.6`, por lo que se tomarían {math}`2` muestras de esa 
partícula. Esto nos asegura que las partículas con probabilidades altas
se toman al menos una vez.  

Sin embargo, esto no nos genera {math}`N` partículas. Por tanto, tenemos que tener en
cuenta los {math}`\textbf{residuos}`, los nuevos pesos menos su parte entera (la parte
decimal de los pesos nuevos) ya que las habíamos obviado. Después, usamos 
un sampling más sencillo como el {math}`\textit{multinomial}` para seleccionar el resto debería
partículas basadas en el residuo. Si continuamos el ejemplo, la parte residual
de nuestra partícula era {math}`2.6- \texttt{int}(2) = 0.6`. El residuo es grande, por
lo que la partícula tiene mucha probabilidad de ser seleccionada de nuevo. 

En el ejemplo siguiente, hemos utilizado {math}`N=15`:

```{image} ../../_static/images/residual_resampling.png
:alt: Residual resampling
:align: center
```

### Stratified Resampling
Este método busca hacer selecciones relativamente uniformes en el conjunto de 
partículas. Funciona de la siguiente manera: dividimos la suma 
acumulada en {math}`N` secciones iguales (estratos) y luego, dentro de cada
sección, escogemos una partícula al azar. Esto nos garantiza que la 
distancia entre dos muestras tomadas {math}`\in [0,\frac{2}{N}]`.   

En la gráfica, vemos los estratos separados por líneas verticales azules, y 
en cada uno de ellos se ha escogido una muestra aleatoria. 

````{image} ../../_static/images/stratified_resampling.png
:alt: Stratified resampling
:align: center
````

### Systematic Resampling
En este algoritmo, como en el estratificado, se divide la suma acumulada en 
{math}`N` secciones (que están a distancia {math}`\frac{1}{N}`). Luego, la idea es escoger 
una partícula aleatoria e ir tomando de las demás secciones las que estén 
a una distancia proporcional a {math}`\frac{1}{N}` de ella. 

```{image} ../../_static/images/systematic_resampling.png
:alt: Systematic resampling
:align: center
```
