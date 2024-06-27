#!/usr/bin/env python3

import rosbag
import matplotlib.pyplot as plt
import os
from tkinter import Tk, filedialog
from car_state import CarState  

def extract_data_from_bag(bag, topic):
    times = []
    data = []

    for _, msg, t in bag.read_messages(topics=[topic]):
        times.append(t.to_sec())
        data.append(msg)
    
    return times, data

def plot_data(times, data, label, xlabel, ylabel, title, subplot_position):
    plt.subplot(subplot_position)
    plt.plot(times, data, label=label)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend()

def main():
    # Selección de archivo utilizando una interfaz gráfica
    Tk().withdraw()  # Para ocultar la ventana principal de Tkinter
    bag_file = filedialog.askopenfilename(filetypes=[("ROS bag files", "*.bag")])
    
    if not bag_file:
        print("No file selected. Exiting.")
        return
    
    bag = rosbag.Bag(bag_file)
    
    plots = []
    try:
        times, messages = extract_data_from_bag(bag, '/car_state')
        speeds = [msg.speed for msg in messages]
        positions_x = [msg.x for msg in messages]
        positions_y = [msg.y for msg in messages]
        accelerations = [msg.acceleration for msg in messages]
        velocities = [msg.velocity for msg in messages]

        plots.extend([
            (times, speeds, 'Speed', 'Time [s]', 'Speed [m/s]', 'Speed vs Time', 321),
            (times, positions_x, 'Position X', 'Time [s]', 'Position X [m]', 'Position X vs Time', 322),
            (times, positions_y, 'Position Y', 'Time [s]', 'Position Y [m]', 'Position Y vs Time', 323),
            (times, accelerations, 'Acceleration', 'Time [s]', 'Acceleration [m/s²]', 'Acceleration vs Time', 324),
            (times, velocities, 'Velocity', 'Time [s]', 'Velocity [m/s]', 'Velocity vs Time', 325),
            (positions_x, positions_y, 'Trajectory', 'Position X [m]', 'Position Y [m]', 'Car Trajectory', 326)
        ])
    except Exception as e:
        print(f"No hay datos de /car_state: {e}")

    try:
        times, messages = extract_data_from_bag(bag, '/other_state')
        other_data = [msg.some_property for msg in messages]
        
        plots.append((times, other_data, 'Other Data', 'Time [s]', 'Other Data', 'Other Data vs Time', 327))
    except Exception as e:
        print(f"No hay datos de /other_state: {e}")

    plt.figure(figsize=(15, 10))
    
    for plot in plots:
        times, data, label, xlabel, ylabel, title, subplot_position = plot
        plot_data(times, data, label, xlabel, ylabel, title, subplot_position)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
