# Can Node

```{warning}
It's  necessary to handle with care to avoid the execution of unwanted commands to the inverter.
```

## Introduction
This node is designed to handle communication with a CAN bus, specifically to control and monitor the motor through our ROS system.
The script runs in a main asynchronous loop, where it manages the subscription to ROS topics and the publication of the motor speed. It includes a callback system to process received messages and an asynchronous method to collect and publish motor speed data based on CAN communication.

## Main features
- **CAN communication**: The node is responsible for sending and receiving CAN messages through the `can0` interface of the operating system.
- **Asynchrony**: It uses an asynchronous loop to continuously and efficiently manage communication and data processing.
- **Execution Security**: It includes comments and warnings to avoid the accidental execution of critical commands.

```{tip}
For more information about the operation of CAN communication, see the [Can Connection](can_connection.md) page.
```
```{tip}
For more information about the inverter, download the [Bamocar manual](https://usermanual.wiki/Document/Bamocar20CAN20Manual.43600820/view).
```

