### Pasos para conseguir la matriz de homografía de la cámara
1. Ejecutar el script _saycheese.py_ para hacer foto de los conos con la disposición que aparece en la [imagen](disposition.png) y después grabar el vídeo del tablero de ajedrez.
2. Cambiar el archivo conos.txt con las coordenadas de los conos con 0 en la variable z (no usar comas para separar las coordenadas).
3. Ejecutar el script calibration.py
4. La matriz de homografía se guardará en el archivo mathom.txt, la matriz de intrínsecas en matint.txt.
---
### Para ejecutar en un nuevo equipo
- Cambiar también la variable **cam_index** el script _saycheese.py_ para que coincida con el índice de la cámara conectada y, si es necesario, cambiar el tiempo que dura la grabación.
- Cambiar la ruta en el archivo _cones.data_ de la carpeta _weights_ para que coincida con el del equipo.
- Añadir el archivo _libdarknet_.so a la carpeta darknet.