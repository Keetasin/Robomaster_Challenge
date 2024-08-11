import time
import csv
import matplotlib.pyplot as plt
from robomaster import robot

# Global variables
current_x = 0.0
position_data = []
tof_data = []
position_time_data = []
tof_time_data = []
left_data = []
right_data = []
left_time_data = []
right_time_data = []

def sub_position_handler(position_info):
    global current_x
    x, y, z = position_info
    current_x = x
    position_data.append(position_info)
    position_time_data.append(time.time())
    print(f"Position: x={x}, y={y}, z={z}")

def sub_tof_handler(tof_info):
    tof_data.append(tof_info[0])
    tof_time_data.append(time.time())
    print(f"TOF: distance={tof_info[0]}")

def sub_data_handler(sub_info):
    io_data, ad_data = sub_info
    
    # Convert each ADC value to voltage and calculate distance
    distances = []
    for adc_value in ad_data:
        voltage = adc_value * 3.3 / 1023

        # Define the piecewise linear approximation ranges and coefficients
        ranges = [
            {'m': -0.3846, 'c': 4.30764, 'min': 2.2, 'max': 3.2},
            {'m': -0.2, 'c': 3.2, 'min': 1.4, 'max': 2.2},
            {'m': -0.067, 'c': 1.87, 'min': 0.8, 'max': 1.4},
            {'m': -0.034, 'c': 1.344, 'min': 0.4, 'max': 0.8}
        ]

        distance = None
        for range_ in ranges:
            if range_['min'] <= voltage < range_['max']:
                distance = (voltage - range_['c']) / range_['m']
                distances.append(distance)
                break
    
    # Calculate avg for left and right sensor
    left = sum(distances[0:2]) / 2
    right = sum(distances[2:4]) / 2
    left_data.append(left)
    right_data.append(right)
    left_time_data.append(time.time())
    right_time_data.append(time.time())

    # print(f"io value: {io_data}, ad values: {ad_data}")
    # print(f"Distance to the foam wall: {distances} cm")
    print(f"port1 left: {left}, port2 right: {right}")
    return distances

def move_until_tof_less_than(ep_chassis, threshold_distance, max_time, overall_start_time, time_data, list_current_x):
    start_time = time.time()

    while (time.time() - start_time) < max_time:
        if tof_data and tof_data[-1] < threshold_distance:
            break
        
        ep_chassis.drive_wheels(w1=100, w2=100, w3=100, w4=100)  # Adjust speed as needed
        list_current_x.append(current_x)
        time_data.append(time.time() - overall_start_time)
        
        time.sleep(0.1)  # Control frequency

    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    time.sleep(0.5)

def rotate_180_degrees(ep_chassis):
    ep_chassis.move(x=0, y=0, z=180, xy_speed=20).wait_for_completed()
    time.sleep(0.5)

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor
    ep_sensor_adaptor = ep_robot.sensor_adaptor
    ep_gimbal = ep_robot.gimbal

    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_sensor.sub_distance(freq=10, callback=sub_tof_handler)
    ep_sensor_adaptor.sub_adapter(freq=10, callback=sub_data_handler)  # Subscribe to analog data
    time.sleep(1)

    time_data, list_current_x = [], []
    overall_start_time = time.time()

    ep_gimbal.recenter().wait_for_completed()
    time.sleep(0.5)
    
    for _ in range(4):
        move_until_tof_less_than(ep_chassis, 500, 10, overall_start_time, time_data, list_current_x)  # Adjust max_time as needed
        rotate_180_degrees(ep_chassis)
        ep_gimbal.recenter().wait_for_completed()
        time.sleep(0.5)

    ep_chassis.unsub_position()
    ep_sensor.unsub_distance()
    ep_sensor_adaptor.unsub_adapter()
    ep_robot.close()

    # Save data to CSV
    with open('chassis_data.csv', 'w', newline='') as csvfile:
        fieldnames = ['timestamp', 'position_x', 'position_y', 'position_z', 'tof_distance', 'left_data', 'right_data']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for idx in range(max(len(position_time_data), len(tof_time_data), len(left_time_data), len(right_time_data))):
            row = {'timestamp': position_time_data[idx] if idx < len(position_time_data) else '',
                   'position_x': position_data[idx][0] if idx < len(position_data) else '',
                   'position_y': position_data[idx][1] if idx < len(position_data) else '',
                   'position_z': position_data[idx][2] if idx < len(position_data) else '',
                   'tof_distance': tof_data[idx] if idx < len(tof_data) else '',
                   'left_data': left_data[idx] if idx < len(left_data) else '',
                   'right_data': right_data[idx] if idx < len(right_data) else ''}
            writer.writerow(row)

    # Plotting data
    plt.figure()

    plt.subplot(2, 2, 1)
    if position_time_data:
        time_series = [t - position_time_data[0] for t in position_time_data]
        plt.plot(time_series, [d[0] for d in position_data], label='X')
        plt.plot(time_series, [d[1] for d in position_data], label='Y')
    plt.title("Position Data")
    plt.legend()

    plt.subplot(2, 2, 2)
    if tof_time_data:
        time_series = [t - tof_time_data[0] for t in tof_time_data]
        plt.plot(time_series, tof_data, label='Distance')
    plt.title("TOF Data")
    plt.legend()

    plt.subplot(2, 2, 3)
    if left_time_data:
        time_series = [t - left_time_data[0] for t in left_time_data]
        plt.plot(time_series, left_data, label='Left Sensor')
        plt.plot(time_series, right_data, label='Right Sensor')
    plt.title("Left/Right Sensor Data")
    plt.legend()

    plt.tight_layout()
    plt.show()
