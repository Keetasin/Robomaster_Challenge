import time
import csv
import matplotlib.pyplot as plt
import numpy as np
from robomaster import robot

# Global variables
current_x = 0.0
current_y = 0.0
position_data = []
tof_data = []
position_time_data = []
tof_time_data = []
left_data = []
right_data = []
left_time_data = []
right_time_data = []
left = 0.0
right = 0.0

# Grid initialization
grid_size = 6
grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

def sub_position_handler(position_info):
    global current_x, current_y
    x, y, z = position_info
    current_x = x
    current_y = y
    position_data.append(position_info)
    position_time_data.append(time.time())
    # Update the grid
    grid_x = int(current_x / 0.55)  # Assuming x is in meters
    grid_y = int(current_y / 0.55)  # Assuming y is in meters
    if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
        grid[grid_y][grid_x] = 1
    print_grid()

def sub_tof_handler(tof_info):
    tof_data.append(tof_info[0])
    tof_time_data.append(time.time())
    
def sub_data_handler(sub_info):
    io_data, ad_data = sub_info
    
    distances = []
    for adc_value in ad_data:
        voltage = adc_value * 3.3 / 1023

        # Adjusted piecewise linear approximation
        if 2.2 <= voltage < 3.2:
            distance = (voltage - 4.30764) / -0.3846
        elif 1.4 <= voltage < 2.2:
            distance = (voltage - 3.2) / -0.2
        elif 0.8 <= voltage < 1.4:
            distance = (voltage - 1.87) / -0.067
        elif 0.4 <= voltage < 0.8:
            distance = (voltage - 1.344) / -0.034
        else:
            distance = None  # Voltage out of expected range

        if distance:
            distances.append(distance)

    if len(distances) == 4:
        left = sum(distances[0:2]) / 2
        right = sum(distances[2:4]) / 2
        left_data.append(left)
        right_data.append(right)
        left_time_data.append(time.time())
        right_time_data.append(time.time())

        print(f"port1 left: {left}, port2 right: {right}")
    else:
        print(f"Warning: Unexpected voltage value outside of defined ranges.")
    
    return distances

def move_until_tof_less_than(ep_chassis, threshold_distance, overall_start_time, time_data, list_current_x):
    global left, right

    while True:
        if tof_data and tof_data[-1] < threshold_distance:
            break
        
        # if right <= 10 and right > 0:
        #     ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
        #     ep_chassis.drive_wheels(w1=15, w2=-15, w3=15, w4=-15)  
        #     print('<')
        # elif left <= 10:
        #     ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
        #     ep_chassis.drive_wheels(w1=-15, w2=15, w3=-15, w4=15)  
        #     print('>')
        # else:
        ep_chassis.drive_wheels(w1=90, w2=90, w3=90, w4=90)  

        list_current_x.append((current_x, current_y))
        time_data.append(time.time() - overall_start_time)
        
        time.sleep(0.1)  # Control frequency

    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    time.sleep(0.5)

def rotate_180_degrees(ep_chassis):  
    ep_chassis.move(x=0, y=0, z=180, xy_speed=15).wait_for_completed()
    time.sleep(0.5)

def rotate_left(ep_chassis):  
    ep_chassis.move(x=0, y=0, z=90, xy_speed=15).wait_for_completed()
    time.sleep(0.5)

def rotate_right(ep_chassis):  
    ep_chassis.move(x=0, y=0, z=-90, xy_speed=15).wait_for_completed()
    time.sleep(0.5)

def print_grid():
    for row in grid:
        print(' '.join(str(cell) for cell in row))
    print('-' * (grid_size * 2 - 1))

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

    while True:
        move_until_tof_less_than(ep_chassis, 400, overall_start_time, time_data, list_current_x)  
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
        time.sleep(0.5)

        # while True:
        #     if right <= 10 and right > 0:
        #         ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
        #         ep_chassis.drive_wheels(w1=15, w2=-15, w3=15, w4=-15) 
        #         print('<-') 
        #     elif left <= 10:
        #         ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
        #         ep_chassis.drive_wheels(w1=-15, w2=15, w3=-15, w4=15)  
        #         print('->') 
        #     else:
        #         print(right, left)
        #         print('ok')
        #         print('-'*10)
        #         break  
        # time.sleep(0.5)

        ep_gimbal.moveto(pitch=0, yaw=90, pitch_speed=0, yaw_speed=30).wait_for_completed()
        time.sleep(0.2)

        if tof_data and tof_data[-1] >= 700:
            rotate_right(ep_chassis)
        else:
            ep_gimbal.recenter().wait_for_completed()
            time.sleep(0.2)
            ep_gimbal.moveto(pitch=0, yaw=-90, pitch_speed=0, yaw_speed=30).wait_for_completed()
            time.sleep(0.2)
            if tof_data and tof_data[-1] >= 700:
                rotate_left(ep_chassis)
            else:
                ep_gimbal.recenter().wait_for_completed()
                time.sleep(0.2)
                rotate_180_degrees(ep_chassis)

        ep_gimbal.recenter().wait_for_completed()
        time.sleep(0.2)

        if tof_data and tof_data[-1] >= 1000 and right >= 18 and left >= 18: break

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