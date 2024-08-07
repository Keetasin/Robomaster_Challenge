# หมุน180
# gimbal หน้าตลอด
# เเปลงsharp
# เอาsensor กับ  moveมาาทำงานร่วมกัน
# digital filter

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

def sub_data_handler_left_right(sub_info):
    io_data, ad_data = sub_info
    left = sum(ad_data[0:2])/2
    right = sum(ad_data[2:4])/2
    left_data.append(left)
    right_data.append(right)
    left_time_data.append(time.time())
    right_time_data.append(time.time())
    print(f"port1 left: {left}, port2 right: {right}")

def move_distance(ep_chassis, target_distance, kp, ki, kd, tolerance, max_time, overall_start_time, time_data, list_current_x, target):
    start_time = time.time()
    prev_error = 0.0
    integral = 0.0
    prev_time = start_time

    while abs(target_distance - current_x) > tolerance and (time.time() - start_time) < max_time:
        current_time = time.time()
        error = target_distance - current_x
        time_diff = current_time - prev_time
        integral += error * time_diff
        derivative = (error - prev_error) / time_diff if time_diff > 0 else 0.0
        speed = kp * error + kd * derivative + ki * integral
        speed = max(min(speed, 150), -150)

        ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
        list_current_x.append(current_x)
        target.append(target_distance)
        time_data.append(current_time - overall_start_time)

        prev_error = error
        prev_time = current_time
        time.sleep(0.05)  # Control frequency

    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    time.sleep(0.5)

def stop_and_record(ep_chassis, target_distance, overall_start_time, time_data, list_current_x, target, stop_duration=1):
    stop_time = time.time()
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    while (time.time() - stop_time) < stop_duration:
        list_current_x.append(current_x)
        target.append(target_distance)
        time_data.append(time.time() - overall_start_time)
        time.sleep(0.1)

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor
    ep_sensor_adaptor = ep_robot.sensor_adaptor

    kp, ki, kd = 120, 5, 30
    tolerance = 0.01
    max_time = 7
    rounds = 2

    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_sensor.sub_distance(freq=10, callback=sub_tof_handler)
    ep_sensor_adaptor.sub_adapter(freq=10, callback=sub_data_handler_left_right)  # Subscribe to analog data
    time.sleep(1)

    time_data, list_current_x, target = [], [], []
    overall_start_time = time.time()

    for _ in range(rounds):
        move_distance(ep_chassis, 1.5, kp, ki, kd, tolerance, max_time, overall_start_time, time_data, list_current_x, target)
        stop_and_record(ep_chassis, 1.5, overall_start_time, time_data, list_current_x, target)

        move_distance(ep_chassis, 0.0, kp, ki, kd, tolerance, max_time, overall_start_time, time_data, list_current_x, target)
        stop_and_record(ep_chassis, 0.0, overall_start_time, time_data, list_current_x, target)

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
