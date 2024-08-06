import rosbag
import pandas as pd
import matplotlib.pyplot as plt
import sys
import numpy as np

# def plot_gps(bag_path):
#     """Reads GPS data from /GPS_location and plots latitude vs. longitude."""
#     print(f"Procesando datos GPS desde {bag_path}")
#     bag = rosbag.Bag(bag_path)
#     topic_of_interest = '/GPS_location'
#     data = {'time': [], 'latitude': [], 'longitude': []}

#     for topic, msg, t in bag.read_messages(topics=[topic_of_interest]):
#         data['time'].append(t.to_sec())
#         data['latitude'].append(msg.latitude)
#         data['longitude'].append(msg.longitude)

#     bag.close()
#     df = pd.DataFrame(data)

#     return df

def plot_AS_status(bag_path):
    print(f"Procesando datos AS Status desde {bag_path}")
    bag = rosbag.Bag(bag_path)
    topic_of_interest = '/can/AS_status'
    data = {'time': [],'AS_status': []}

    for topic, msg, t in bag.read_messages(topics=[topic_of_interest]): 
        data['time'].append(t.to_sec())
        data['AS_status'].append(msg.data)

    bag.close()
    df = pd.DataFrame(data)

    return df

def plot_linear_acc(bag_path):
    print(f"Procesando datos linear_velocity desde {bag_path}")
    bag = rosbag.Bag(bag_path)
    topic_of_interest = '/can/IMU'
    data = {'time': [], 'Linear_acceleration': []}
    
    asdf = 0
    for topic, msg, t in bag.read_messages(topics=[topic_of_interest]): 
        data['time'].append(t.to_sec())
        asdf = asdf*0.9 + 0.1*msg.linear_acceleration.x
        data['Linear_acceleration'].append(asdf)

    bag.close()
    df = pd.DataFrame(data)

    return df

def plot_car_state(bag_path):
    """Reads car_state data from /car_state/state and plots position (X, Y) colored by velocity (vx)
       and motor speed (vx)."""
    print(f"Procesando datos del estado del coche desde {bag_path}")
    bag = rosbag.Bag(bag_path)
    topic_of_interest = '/car_state/state'
    data = {'time': [], 'x': [], 'y': [], 'vx': [], 'vy': []}

    for topic, msg, t in bag.read_messages(topics=[topic_of_interest]):
        data['time'].append(t.to_sec())
        data['x'].append(msg.x)
        data['y'].append(msg.y)
        data['vx'].append(msg.vx)
        data['vy'].append(msg.vy)

    bag.close()
    df = pd.DataFrame(data)

    return df

def plot_steering(bag_path):
    """Reads steering data from /steering/epos_info and plots actual vs. target steering."""
    print(f"Procesando datos de dirección desde {bag_path}")
    bag = rosbag.Bag(bag_path)
    topic_of_interest = '/steering/epos_info'
    data = {'time': [], 'real_steering': [], 'target_steering': []}

    for topic, msg, t in bag.read_messages(topics=[topic_of_interest]):
        data['time'].append(t.to_sec())
        data['real_steering'].append(msg.data[1])
        data['target_steering'].append(msg.data[2])

    bag.close()
    df = pd.DataFrame(data)

    return df

def calculate_deceleration(bag_path):
    print(f"Procesando datos desde {bag_path}")
    bag = rosbag.Bag(bag_path)
    
    # Define topics of interest
    as_status_topic = '/can/AS_status'
    speed_topic = '/can/speed'

    # Initialize data dictionaries
    as_data = {'time': [], 'AS_status': []}
    speed_data = {'time': [], 'speed': []}
    print(speed_data)

    # Read AS status messages
    for topic, msg, t in bag.read_messages(topics=[as_status_topic]):
        as_data['time'].append(t.to_sec())
        as_data['AS_status'].append(msg.data)  # Assuming AS status is in msg.data

    # Read speed messages
    for topic, msg, t in bag.read_messages(topics=[speed_topic]):
        speed_data['time'].append(t.to_sec())
        speed_data['speed'].append(msg.data)  # Assuming speed is in msg.data

    bag.close()

    # Convert to DataFrame for easier handling
    df_as = pd.DataFrame(as_data)
    df_speed = pd.DataFrame(speed_data)

    # Merge data on time
    df = pd.merge_asof(df_as.sort_values('time'), df_speed.sort_values('time'), on='time', direction='forward')
    
    # Identify the index where AS status changes to 4
    start_index = df[df['AS_status'] == 4].index[0]

    # Identify the index where speed is less than 1 m/s after the start_index
    end_index = df[(df.index > start_index) & (df['speed'] < 1)].index[0]

    # Calculate deceleration
    start_time = df.loc[start_index, 'time']
    end_time = df.loc[end_index, 'time']
    start_speed = df.loc[start_index, 'speed']
    end_speed = df.loc[end_index, 'speed']
    deceleration = (start_speed - end_speed) / (end_time - start_time)


def main(bag_path):
    # Read and process data from each bag
    print("Leyendo y procesando datos del archivo .bag...")
    acc_df = plot_linear_acc(bag_path)
    # gps_df = plot_gps(bag_path)
    car_state_df = plot_car_state(bag_path)
    AS_df = plot_AS_status(bag_path)
    steering_df = plot_steering(bag_path)



    # Create a figure with 2x2 subplots
    plt.figure(figsize=(15, 10))



    # Plot linear acceleration in X
    plt.subplot(2,2,1)
    plt.plot(np.array(acc_df)[:,0], np.array(acc_df)[:,1], marker='.', linestyle='-', color='pink', markersize='0.1')
    plt.xlabel('Time')
    plt.ylabel('Linear Acceleration')
    plt.title('Linear acceleration in X')
    plt.grid(True)


    # # Plot GPS data
    # plt.subplot(2, 2, 1)
    # plt.plot(gps_df['longitude'], gps_df['latitude'], marker='.', linestyle='-', color='b')
    # plt.xlabel('Longitude')
    # plt.ylabel('Latitude')
    # plt.title('GPS Data (Latitude and Longitude)')
    # plt.grid(True)

    # Plot position (X, Y) colored by velocity (vx)
    plt.subplot(2, 2, 2)
    sc = plt.scatter(np.array(car_state_df)[:,1], np.array(car_state_df)[:,2], c=np.array(car_state_df)[:,3], cmap='viridis', marker='o')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.colorbar(sc, label='Velocity (vx)')
    plt.title('Position (X, Y) colored by Velocity (vx)')
    plt.grid(True)

    # Plot car state: motor speed (vx)
    plt.subplot(2, 2, 3)
    plt.plot(np.array(car_state_df)[:,0], np.array(car_state_df)[:,3], label= 'Car State', marker='.', linestyle='-', color='b', markersize='0.1')
    plt.plot(np.array(AS_df)[:,0], np.array(AS_df)[:,1], label='AS Status', marker='.', linestyle='-', color='r', markersize='0.1')
    plt.xlabel('Time (s)')
    # plt.ylabel('Speed (m/s)')
    plt.title('Car State: Motor Speed (vx) & AS Status')
    plt.grid(True)

    # Plot steering: actual vs. target steering
    plt.subplot(2, 2, 4)
    plt.plot(np.array(steering_df)[:,0], np.array(steering_df)[:,1], label='Actual Steering', marker='.', linestyle='-', color='r', markersize='1')
    plt.plot(np.array(steering_df)[:,0], np.array(steering_df)[:,2], label='Target Steering', marker='.', linestyle='-', color='g', markersize='1')
    plt.xlabel('Time (s)')
    plt.ylabel('Steer Angle (rad)')
    plt.title('Steering: Actual vs Target Steering')
    plt.legend()
    plt.grid(True)

    # Tighten layout and adjust spacing between subplots
    plt.tight_layout()
    plt.subplots_adjust(left=0.08, right=0.92, top=0.92, bottom=0.08)

    # Show the combined plot
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python grafiquitas.py <path_to_bag_file>")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    print(f"Archivo .bag proporcionado: {bag_path}")
    main(bag_path)
