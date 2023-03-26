# Environment setup

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

## Setting up git and Github
We mainly use Github during development, being [`ARUSfs/DRIVERLESS`](https://github.com/ARUSfs/DRIVERLESS) the main repository. Some other repositories exist for MCU code or secondary projects that don't quite fit the main repository. Since most repositories are private, during push/pull operations you will need to introduce your credentials. The other option is to save credentials in plain text configuration file, which isn't a good security practice. We detail here how to setup ssh keys to be used with Github, as well as explaining some git basics.

```{note}
We will only cover how to set up ssh keys for development in `Ubuntu/Debian` distros, though the steps will probably be similar in other environments.
```

1. Run `ssh-keygen` in a console. If you aren't sure about the prompts, you can use the default values by pressing `Enter` until the process has finished. This will create two files:
    * `~/.ssh/id_rsa`: You should keep this file secret. Never give it away.
    * `~/.ssh/id_rsa.pub`: This is the *public* key.
2. Open your Github `Settings` page
    1. `SSH and GPG keys` > `New SSH key` 
    2. `Title`: Enter a descriptive name for your computer.
    3. `Key`: Copy all the contents of `~/.ssh/id_rsa.pub` and paste in this field.
    4. `Add SSH key`
3. Try to only keep keys that you are certain are safe and remove any that you know you won't use again. Anyone with your private key could impersonate you!
4. To make sure everything works correctly, you can run `ssh -T git@github.com`. Github should greet you back!
5. Now go to wherever you wish to download the repository, and run `git clone ssh://git@github.com/ARUSfs/DRIVERLESS`. **Important** to remember that you shouldn't use the normal link. Always precede `ARUSfs` repositories with `ssh://git@github.com` instead of `https://github.com/`.
