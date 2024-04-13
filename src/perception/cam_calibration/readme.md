Pasos para conseguir la matriz de homografía de la cámara:
1.- Ejecutar el script saycheese.py para hacer foto de los conos con la disposición que aparece en la imagen Disposicion.png y después grabar el vídeo del tablero de ajedrez.
2.- Cambiar el archivo conos.txt con las coordenadas de los conos con 0 en la variable z (no usar comas para separar las coordenadas).
3.- Ejecutar el script calibration.py
4.- La matriz de homografía se guardará en el archivo mathom.txt, la matriz de intrínsecas en matint.txt.

- Cambiar también la variable 'cam_index' el script saycheese.py para que coincida con el índice de la cámara conectada y, si es necesario, cambiar el tiempo que dura la grabación.
- Cambiar la ruta en el archivo _cones.data_ de la carpeta _weights_ para que coincida con el del equipo.
- Añadir el archivo libdarknet.so a la carpeta darknet.