# Environment setup

```{toctree}
:maxdepth: 2
:hidden:

can_connection
epos4_libraries

```

## Setting up git and Github
We mainly use Github during development, being [`ARUSfs/DRIVERLESS`](https://github.com/ARUSfs/DRIVERLESS) the main repository. Some other repositories exist for MCU code or secondary projects that don't quite fit the main repository. Since most repositories are private, during push/pull operations you will need to introduce your credentials. Another option is to save credentials in plain text configuration file, which isn't a good security practice. Here we will detail how to setup ssh keys to be used with Github, which is more secure and convinient than the former options.

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

    Try to only keep keys that you are certain are safe and remove any that you know you won't use again. Anyone with your private key could impersonate you!
3. To make sure everything works correctly, you can run `ssh -T git@github.com`. Github should greet you back!
4. Now go to wherever you wish to download the repository, and run `git clone ssh://git@github.com/ARUSfs/DRIVERLESS`. **Important**: always precede `ARUSfs` repositories with `ssh://git@github.com` instead of `https://github.com/`.
5. Move into the directory and configure your mail and name for `git`. It's recommended to set these as global values, but you may ommit the `--global` argument if for any reason you wish to configure it only for this repository. Run
    * `git config --global user.email "you@example.com"`
    * `git config --global user.name "Your Name"`

## Installation of ROS Noetic

To install Ros Noetic on `Ubuntu 20.04` you may follow their [official tutorial](http://wiki.ros.org/noetic/Installation/Ubuntu). If installing for your computer, I would recommend installing the `ros-noetic-desktop-full` package, which includes `rviz` and other tools for visualization. If installing on a machine for inference it will suffice with `ros-noetic-ros-base`. After the installation, you may:
- Make sure that `setup.bash` is sourced on your `.bashrc`(step 1.5 of the ROS tutorial).
- Install `rosdep`.
- Automatically install necessary packages by running.
    ```{code-block}
    $ rosdep install --from-paths src --ignore-src -r -y
    ```

## Installation of CUDA and cuDNN
> *lasciate ogni speranza, voi ch'entrate*

CUDA is a parallel computing platform and API which allows programming for Nvidia GPUs parallelization capabilities. Some other packages exist which include predesigned functions for some aplications like cuBLAS (Basic Linear Algebra Sobprograms) or cuDNN (Deep Neural Networks). While the first one is usually packaged along CUDA, cuDNN isn't so it must be installed separately.

Even though the *theory* of installing these packages is simple, everyone's experience differs. During installation, various problems will arise. These issues will differ from OS to OS, hardware to hardware, and version to version. My intention for this wiki is to be autocontained, but if for any reason you have to install these and are completely unable to fix the problems, feel free to contact me ([Jacobo Pindado Perea](pindado.jacobo@gmail.com)). Anyway, you can always follow the official [tutorial](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html).

Before the installation of all the packages and files verify your system has a CUDA-capable GPU (`lspci | grep -i nvidia`), a supported version of Linux (`uname -m && cat /etc/*release`) and gcc installed (`gcc --version`).

### Step by step installation:
1. Download [CUDA toolkit](https://developer.nvidia.com/cuda-downloads) selecting your system configuration. My recommendation is selecting the _runfile (local)_ installer type. After downloading the package run `md5sum`.
2. Check the installation running `md5sum <file>` (change \<file> with the name of the downloaded _.run_ file) and compare the result with the official NVIDIA checksums.
3. Finally, add at the end of the _bash.rc_ file the following lines changing \<version> with the installed CUDA version:
    ```# NVIDIA CUDA TOOLKIT
    export PATH=/usr/local/cuda-<version>/bin${PATH:+:${PATH}}
    export LD_LIBRARY_PATH=/usr/local/cuda-<version>/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
    ```
4. Run `source ~/.bashrc`
5. Reboot the system `sudo reboot`
6. Confirm the installation by running `nvcc -V` and `nvidia-smi`

## Compilation of OpenCV

Whenever installing and profiling on a machine with GPU
