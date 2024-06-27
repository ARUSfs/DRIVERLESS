#!/usr/bin/env python3

import rosbag
import matplotlib.pyplot as plt
import sys
from car_state import CarState 

def main(bag_file):
    bag = rosbag.Bag(bag_file)
    
    times = []
    speeds = []
    positions_x = []
    positions_y = []
    accelerations = []
    velocities = []
    # Añadir más listas para otras propiedades del mensaje CarState

    for topic, msg, t in bag.read_messages(topics=['/car_state']):
        times.append(t.to_sec())
        speeds.append(msg.speed)
        positions_x.append(msg.x)
        positions_y.append(msg.y)
        accelerations.append(msg.acceleration)
        velocities.append(msg.velocity)
        # Añade más datos aquí según las propiedades del mensaje CarState

    bag.close()

    plt.figure(figsize=(12, 10))

    # Gráfico de Speed vs Time
    plt.subplot(3, 2, 1)
    plt.plot(times, speeds, label='Speed')
    plt.xlabel('Time [s]')
    plt.ylabel('Speed [m/s]')
    plt.title('Speed vs Time')
    plt.legend()

    # Gráfico de Position X vs Time
    plt.subplot(3, 2, 2)
    plt.plot(times, positions_x, label='Position X')
    plt.xlabel('Time [s]')
    plt.ylabel('Position X [m]')
    plt.title('Position X vs Time')
    plt.legend()

    # Gráfico de Position Y vs Time
    plt.subplot(3, 2, 3)
    plt.plot(times, positions_y, label='Position Y')
    plt.xlabel('Time [s]')
    plt.ylabel('Position Y [m]')
    plt.title('Position Y vs Time')
    plt.legend()

    # Gráfico de Acceleration vs Time
    plt.subplot(3, 2, 4)
    plt.plot(times, accelerations, label='Acceleration')
    plt.xlabel('Time [s]')
    plt.ylabel('Acceleration [m/s²]')
    plt.title('Acceleration vs Time')
    plt.legend()

    # Gráfico de Velocity vs Time
    plt.subplot(3, 2, 5)
    plt.plot(times, velocities, label='Velocity')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.title('Velocity vs Time')
    plt.legend()

    # Gráfico de recorrido del coche (X vs Y)
    plt.subplot(3, 2, 6)
    plt.plot(positions_x, positions_y, label='Trajectory')
    plt.xlabel('Position X [m]')
    plt.ylabel('Position Y [m]')
    plt.title('Car Trajectory')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: plot_car_state.py <bag_file>")
        sys.exit(1)

    bag_file = sys.argv[1]
    main(bag_file)
