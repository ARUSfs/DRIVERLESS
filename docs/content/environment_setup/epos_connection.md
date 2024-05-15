# EPOS Connection

## Introduction
The purpose of this document is to explain how to install the libraries required to connect to Maxon's steering engine through EPOS library. 

## Dependencies
First we need to install the libftd2 library by downloading the compressed folder [here](https://ftdichip.com/wp-content/uploads/2022/07/libftd2xx-x86_64-1.4.27.tgz).


Then we extract the folder and follow the RedMe.txt untill line 73.


After that, download the EPOS libary [here](https://www.maxongroup.com/medias/sys_master/root/8994700394526/EPOS-Linux-Library-En.zip).

Install the library by running install.sh (make sure it has execution permissions):
```bash
chmod +x install.sh
./install.sh
```

## How to connect to the steering engine
Connect to the EPOS controller through USB, clone the DRIVERLESS repository and run the steering node:
```bash
rosrun steering main.py
```





