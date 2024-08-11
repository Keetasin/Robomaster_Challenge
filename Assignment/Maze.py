import time
import csv
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from robomaster import robot

# Global variables
current_x = 0.0
current_y = 0.0
position_data = []
tof_data = []
tof_time_data = []
left_data = []
right_data = []
left_time_data = []
right_time_data = []
current_left = 0.0
current_right = 0.0
x = True
count1 = 0

# Grid initialization

def sub_position_handler(position_info):
    x, y, z = position_info
    # print("chassis position: x:{:.2f}, y:{:.2f}, z:{:.2f}".format(x, y, z))
    position_data.append((round(x,2), (round(y,2)), (round(z,2))))


def sub_tof_handler(tof_info):
    tof_data.append(tof_info[0])
    tof_time_data.append(time.time())

def sub_data_handler(sub_info):
    global current_left, current_right
    io_data, ad_data = sub_info
    
    distances = []
    for adc_value in ad_data:
        voltage = adc_value * 3.3 / 1023
        # print(voltage)
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
            if voltage >= 3.2:
                distance = (voltage - 4.30764) / -0.3846
            elif voltage < 0.4:
                distance = 50.0

        distances.append(distance)

    left = sum(distances[0:2]) / 2
    right = sum(distances[2:4]) / 2
    left_data.append(left)
    right_data.append(right)
    left_time_data.append(time.time())
    right_time_data.append(time.time())
    
    if left >= 24:
        left = 50
    if right >= 24:
        right = 50
    if distance == 0:
        print("!" * 15)
        
    # print("ad_data", ad_data)
    current_right = right
    current_left = left
    # print(f"port1 left: {left}, port2 right: {right}")
    print('right = ', right)

    return distances

def move_until_tof_less_than(ep_chassis, threshold_distance, overall_start_time, time_data, list_current_x):
    global current_left, current_right, count1, x 

    while True:
        if position_data:
            x_vals = [pos[0] for pos in position_data]
            y_vals = [pos[1] for pos in position_data]
            ax.clear()
            ax.plot(y_vals, x_vals, 'ro-', label="Robot Path")
            ax.set_xlabel('X Position (cm)')
            ax.set_ylabel('Y Position (cm)')
            ax.set_title('Real-time Robot Path')
            ax.grid(True)
            plt.draw()
            plt.pause(0.01)  # หน่วงเวลาเพื่อให้กราฟอัปเดตได้ทันที
        print('x = ', x)

        if tof_data and tof_data[-1] < threshold_distance:
            print(tof_data[-1])
            break

        if current_right <= 11 :
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
            ep_chassis.drive_wheels(w1=17, w2=-17, w3=17, w4=-17)  
            print(current_right)
            print('<')
        if current_left <= 11:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
            ep_chassis.drive_wheels(w1=-17, w2=17, w3=-17, w4=17)  
            print(current_left)
            print('>')

        if x  and current_right >= 49:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0) 
            print(current_right)
            print('*')
            break

        else:
            ep_chassis.drive_wheels(w1=55, w2=55, w3=55, w4=55) 


        if count1 == 15:
            x = True
            print(x)
            count1 = 0
            print('re right xxxxxxxxxxxxxx') 
        if count1 >= 1:
            count1 += 1
            print(count1)

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

    # สร้างแผนที่
    plt.ion()  # เปิดโหมด interactive
    fig, ax = plt.subplots()

    while True:

        move_until_tof_less_than(ep_chassis, 350, overall_start_time, time_data, list_current_x)  

        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
        time.sleep(0.5)

        while True:
            if current_right <= 11:
                # ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
                ep_chassis.drive_wheels(w1=17, w2=-17, w3=17, w4=-17) 
                print(current_right)
                print('<-') 
            if current_left <= 11:
                # ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
                ep_chassis.drive_wheels(w1=-17, w2=17, w3=-17, w4=17)  
                print(current_left)
                print('->') 
            else:
                ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
                print(current_right, current_left)
                print('ok')
                print('-'*10)
                break  
        time.sleep(0.5)

        
        ep_gimbal.moveto(pitch=0, yaw=90, pitch_speed=0, yaw_speed=40).wait_for_completed()
        time.sleep(0.2)

        if tof_data and tof_data[-1] >= 700:
            print(tof_data[-1])
            x = False
            count1 +=1
            print(count1)
            rotate_right(ep_chassis)
        else:
            ep_gimbal.recenter().wait_for_completed()
            time.sleep(0.2)
            ep_gimbal.moveto(pitch=0, yaw=-90, pitch_speed=0, yaw_speed=40).wait_for_completed()
            time.sleep(0.2)
            if tof_data and tof_data[-1] >= 700:
                print(tof_data[-1])
                x = False
                count1 +=1
                print(count1)
                rotate_left(ep_chassis)
            else:
                ep_gimbal.recenter().wait_for_completed()
                time.sleep(0.2)
                rotate_180_degrees(ep_chassis)
       

        ep_gimbal.recenter().wait_for_completed()
        time.sleep(0.2)

        if tof_data and tof_data[-1] >= 5000 and current_left >= 49 and current_left >= 49: 
            print(tof_data[-1], current_right, current_left)
            break

    ep_chassis.unsub_position()
    ep_sensor.unsub_distance()
    ep_sensor_adaptor.unsub_adapter()
    ep_robot.close()

    # Save data to CSV
    with open('chassis_data.csv', 'w', newline='') as csvfile:
        fieldnames = ['timestamp', 'position_x', 'position_y', 'position_z', 'tof_distance', 'left_data', 'right_data']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for idx in range(max(len(tof_time_data), len(left_time_data), len(right_time_data))):
            row = {'position_x': position_data[idx][0] if idx < len(position_data) else '',
                   'position_y': position_data[idx][1] if idx < len(position_data) else '',
                   'position_z': position_data[idx][2] if idx < len(position_data) else '',
                   'tof_distance': tof_data[idx] if idx < len(tof_data) else '',
                   'left_data': left_data[idx] if idx < len(left_data) else '',
                   'right_data': right_data[idx] if idx < len(right_data) else ''}
            writer.writerow(row)

    # Plotting data
    plt.figure()


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



