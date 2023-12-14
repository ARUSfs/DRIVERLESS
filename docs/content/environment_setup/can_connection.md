# Can Connection

## Introduction
The purpose of this document is to explain how to connect to the car's CAN network. This is necessary in order to send and receive CAN messages from the computer. In the car, the CAN network is connected to a USB port, so to connect to it we need a CAN-USB cable. In the computer, the CAN network is connected through a virtual network interface called `can0`.

## Dependencies
In order to connect to the car's CAN network, we need to install the following dependencies:
```bash
sudo apt install can-utils
pip install python-can
```

## How to connect to the car's CAN network
```{warning}
Before connecting the CAN cable to the car, contact the electronics department to find out how to do it.
```
1. Connect the CAN cable to the car and the computer.
2. Run the following command to bring up the `can0` interface at a speed of 500kbps:
```bash
sudo ip link set can0 up type can bitrate 500000
```
3. Check that the interface has been brought up correctly:
```bash
ifconfig can0
```

```{tip}
This process is performed every time the CAN cable is connected to the computer. If the CAN cable is disconnected, the `can0` interface will be automatically shut down. To avoid having to run the above command every time the CAN cable is connected, you can follow the steps in the [Automatic USB-CAN configuration](#automatic-usb-can-configuration) section.
```

## Automatic USB-CAN configuration
To automatically configure the USB-CAN interface when it is connected to the computer, we can use `udev` rules. These rules are located in `/etc/udev/rules.d/`. We can create a new rule by creating a new file in this directory. For example, we can create a file called `99-usb-can.rules` with the following contents:
```bash
action=="add", SUBSYSTEM=="net", KERNEL=="can*", RUN+="/sbin/ip link set $name type can bitrate 500000"
action=="remove", SUBSYSTEM=="net", KERNEL=="can*", RUN+="/sbin/ip link set $name down"
```
This rule will automatically configure the `can0` interface when the USB-CAN cable is connected to the computer, and will automatically shut it down when the cable is disconnected.



