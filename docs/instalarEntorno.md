# Instalación de entorno

```{warning}
This documentation is under development
```

## Instalar ROS Noetic

- Para instalar ROS Noetic, se debe seguir el tutorial de instalación de ROS Noetic en Ubuntu 20.04 Focal Fossa, disponible en [este enlace](http://wiki.ros.org/noetic/Installation/Ubuntu).
- Es importante tener en cuenta que se debe instalar la versión de ROS Noetic Desktop-Full, ya que es la que contiene todos los paquetes necesarios para el proyecto.
- También es importante usar la versión de Ubuntu 20.04 Focal Fossa, ya que es la que soporta ROS Noetic.

## Instalar paquetes de ROS
Para instalar los paquetes de ROS necesarios para el proyecto, se debe ejecutar el siguiente comando:

```{code-block}
---
emphasize-lines: 1
---
$ rosdep install --from-paths src --ignore-src -r -y
```

