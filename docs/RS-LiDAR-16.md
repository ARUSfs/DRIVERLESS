# RS-LiDAR-16

The RS-LiDAR-16 is Real-Time 3D LiDAR sensor launched by RoboSense. The full specifications can be found [here](https://cdn.robosense.cn/20200723161715_42428.pdf).

```{figure} https://ae01.alicdn.com/kf/H2b50d6cd6c6d437ca1f57f012aaedd66c.jpg_640x640Q90.jpg_.webp
:width: 130 px
:alt: RS-LiDAR-16
RS-LiDAR-16
```

## Como conectar a ROS Noetic
Para conectar el RS-Lidar-16 a nuestro computador, deberemos conectar el LiDAR a su controladora, y luego conectar la controladora a nuestro computador. Para conectar la controladora a nuestro computador, deberemos seguir los siguientes pasos:
 - Conectar el cable ethernet de la controladora a nuestro computador. 
 - Configurar la dirección IP de nuestro computador a `192.168.1.102` y la máscara de subred a `255.255.255.0`. Para esto, podemos seguir los siguientes pasos:
    - En Ubuntu, ir a `Settings > Network > Wired > Options > IPv4 Settings > Method: Manual`. Luego, ingresar la dirección IP y máscara de subred mencionadas anteriormente.
 - Clonar el repositorio [ros_rslidar](https://github.com/RoboSense-LiDAR/rslidar_sdk) en la carpeta `src` de un workspace de trabajo de ROS.
    - En el directorio src del workspace, ejecutar:
    ```bash
    git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
    cd rslidar_sdk
    git submodule init
    git submodule update
    ```
 - Instalación de dependencias:
    - Yaml (Essential)
    ```bash
    sudo apt-get update
    sudo apt-get install -y libyaml-cpp-dev
    ```
    - libpcap (Essential)
    ```bash
    sudo apt-get update
    sudo apt-get install -y libpcap-dev
    ```
 - Editar el archivo `rslidar_sdk/config/config.yaml` y cambiar el valor de `lidar_type` a `RS16`.
 - Compilar el workspace. En la carpeta `src` del workspace, ejecutar:
    ```bash
    catkin_make
    source devel/setup.bash
    roslaunch rslidar_sdk rslidar.launch
    ```
```{attention}
Si ocurre algún error, revise el [Readme oficial de ros_rslidar](https://github.com/RoboSense-LiDAR/rslidar_sdk/blob/main/README.md).
```


