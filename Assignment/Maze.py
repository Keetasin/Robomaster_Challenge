import time
# import csv
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from robomaster import robot

# Global variables
current_x = 0.0
current_y = 0.0
position_data = []
tof_data = []
# tof_time_data = []
left_data = []
right_data = []
# left_time_data = []
# right_time_data = []
current_left = 0.0
current_right = 0.0
x = True
count1 = 0
yaw = None

# Grid initialization

def sub_position_handler(position_info):
    x, y, z = position_info
    # print("chassis position: x:{:.2f}, y:{:.2f}, z:{:.2f}".format(x, y, z))
    position_data.append((round(x,2), (round(y,2)), (round(z,2))))

def sub_attitude_info_handler(attitude_info):
    global yaw
    yaw, pitch, roll = attitude_info
    # print("chassis attitude: yaw:{0}, pitch:{1}, roll:{2} ".format(yaw, pitch, roll))


def sub_tof_handler(tof_info):
    tof_data.append(tof_info[0])
    # tof_time_data.append(time.time())


def filter_adc_value(ad_data):
    y_filtered = []
    alpha = 0.1  # ตั้งค่า alpha ตามความต้องการ
    y_prev = 0  # ค่าเริ่มต้นของ y[n-1]

    for adc_value in ad_data: # 153, 153, 203, 203
        y_current = alpha * y_prev + (1 - alpha) * adc_value
        y_filtered.append(y_current)
        y_prev = y_current

    return y_filtered # 150, 150, 200, 200

def sub_data_handler(sub_info):
    global current_left, current_right
    io_data, ad_data = sub_info
    
    # กรองค่า ADC
    y_filtered = filter_adc_value(ad_data)

    distances = []
    for adc_filtered in  y_filtered:
        voltage = adc_filtered * 3.3 / 1023

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
                distance = (voltage - 1.344) / -0.034

        distances.append(distance)
    
    sharp_left = sum(distances[0:2]) / 2
    sharp_right = sum(distances[2:4]) / 2
    left_data.append(sharp_left)
    right_data.append(sharp_right)
    
    # Error left 
    if sharp_left >= 13:
        sharp_left += 2
    
    # ตัดช่วง
    if sharp_left >= 20:
        sharp_left = 50
    if sharp_right >= 26:
        sharp_right = 50

    current_right = sharp_right
    current_left = sharp_left

    # print(f"PORT2 right: {sharp_right}")
    # print(f"PORT1 left: {sharp_left}, PORT2 right: {sharp_right}")
    
    return distances

def move_until_tof_less_than(ep_chassis, threshold_distance, overall_start_time, time_data, list_current_x):
    global current_left, current_right, count1, x 

    while True:
        # if position_data:
        #     x_vals = [pos[0] for pos in position_data]
        #     y_vals = [pos[1] for pos in position_data]
        #     ax.clear()
        #     ax.plot(y_vals, x_vals ,'*-', label="Robot Path")
        #     ax.set_xlabel('X Position (cm)')
        #     ax.set_ylabel('Y Position (cm)')
        #     ax.set_title('Real-time Robot Path')
        #     ax.grid(True)
        #     plt.draw()
        #     plt.pause(0.01)  # หน่วงเวลาเพื่อให้กราฟอัปเดตได้ทันที
        

        if tof_data and tof_data[-1] < threshold_distance:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
            print('TOF = ', tof_data[-1],'+++++++++++++')
            count1 = 0
            print(count1)
            break

        elif x  and current_right >= 49:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0) 
            # ep_chassis.move(x=0.07, y=0, z=0, xy_speed=0.1).wait_for_completed()
            # ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0) 
            time.sleep(0.05)
            print(current_right)
            print('*****************************')
            break

        elif current_right <= 10 :
            # ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
            ep_chassis.drive_wheels(w1=15, w2=-15, w3=15, w4=-15)  
            print(current_right)
            print('<')
        elif current_left <= 10:
            # ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
            ep_chassis.drive_wheels(w1=-15, w2=15, w3=-15, w4=15)  
            print(current_left)
            print('>')

        else:
            ep_chassis.drive_wheels(w1=50, w2=50, w3=50, w4=50) 


        if count1 == 15:
            x = True
            print(x)
            print('x = ', x)
            count1 = 0
            print('re right xxxxxxxxxxxxxx') 
        if count1 >= 1:
            count1 += 1
            print(count1)
            print('x = ', x)

        list_current_x.append((current_x, current_y))
        time_data.append(time.time() - overall_start_time)
        
        time.sleep(0.1)  # Control frequency

    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    time.sleep(0.1)

def rotate_180_degrees(ep_chassis):  
    ep_chassis.move(x=0, y=0, z=180, z_speed=80).wait_for_completed()
    time.sleep(0.1)

def rotate_left(ep_chassis):  
    ep_chassis.move(x=0, y=0, z=90, z_speed=80).wait_for_completed()
    time.sleep(0.1)

def rotate_right(ep_chassis):  
    ep_chassis.move(x=0, y=0, z=-90, z_speed=80).wait_for_completed()
    time.sleep(0.1)


def adjust_angle(yaw):
    target_yaw = 0
    correction = yaw

    if -135 < yaw <= -45:
        target_yaw = -90
        ep_chassis.move(x=0, y=0, z=correction-target_yaw, z_speed=20).wait_for_completed()  
    elif 45 < yaw < 135:
        target_yaw = 90
        ep_chassis.move(x=0, y=0, z=correction-target_yaw, z_speed=20).wait_for_completed()
    elif -45 < yaw <= 45:
        target_yaw = 0
        ep_chassis.move(x=0, y=0, z=correction, z_speed=20).wait_for_completed()
    elif -180 <= yaw < -135 :
        target_yaw = -180
        ep_chassis.move(x=0, y=0, z=correction-target_yaw, z_speed=20).wait_for_completed() 
    elif 135 < yaw <= 180:
        target_yaw = 180
        ep_chassis.move(x=0, y=0, z=correction-target_yaw, z_speed=20).wait_for_completed() 


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor
    ep_sensor_adaptor = ep_robot.sensor_adaptor
    ep_gimbal = ep_robot.gimbal

    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)
    ep_sensor.sub_distance(freq=10, callback=sub_tof_handler)
    ep_sensor_adaptor.sub_adapter(freq=10, callback=sub_data_handler)  # Subscribe to analog data
    time.sleep(0.2)

    time_data, list_current_x = [], []
    overall_start_time = time.time()

    ep_gimbal.recenter().wait_for_completed()
    time.sleep(0.1)

    # สร้างแผนที่
    # plt.ion()  # เปิดโหมด interactive
    # fig, ax = plt.subplots()

    while True:

        move_until_tof_less_than(ep_chassis, 300, overall_start_time, time_data, list_current_x)  

        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
        time.sleep(0.1)

        while True:
            if tof_data[-1] < 190:
                ep_chassis.drive_wheels(w1=-20, w2=-20, w3=-20, w4=-20) 
                print(tof_data[-1])
                print('back')

            elif tof_data[-1] > 310 and tof_data[-1] < 410:
                ep_chassis.drive_wheels(w1=20, w2=20, w3=20, w4=20) 
                print(tof_data[-1])
                print('forword')
            
            elif current_right <= 10:
                # ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
                ep_chassis.drive_wheels(w1=15, w2=-15, w3=15, w4=-15) 
                print(current_right)
                print('<-') 

            elif current_left <= 10:
                # ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
                ep_chassis.drive_wheels(w1=-15, w2=15, w3=-15, w4=15)  
                print(current_left)
                print('->') 

            else:
                ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
                print(current_right, current_left)
                print('ok')
                print('-'*10)
                break  
        time.sleep(0.2)

        
        # ep_gimbal.moveto(pitch=0, yaw=90, pitch_speed=0, yaw_speed=200).wait_for_completed()
        # time.sleep(0.35)

        # if tof_data and tof_data[-1] >= 600:
        #     print(tof_data[-1])
        #     x = False
        #     count1 +=1
        #     print(count1)
        #     rotate_right(ep_chassis)

        if current_right >= 49:
                print('turn right ', current_right)
                x = False
                count1 +=1
                print(count1)
                rotate_right(ep_chassis)

        else:
            ep_gimbal.recenter(yaw_speed=200).wait_for_completed()
            time.sleep(0.1)
            ep_gimbal.moveto(pitch=0, yaw=-90, pitch_speed=0, yaw_speed=200).wait_for_completed()
            time.sleep(0.35)
            if tof_data and tof_data[-1] >= 600:
                print(tof_data[-1])
                x = False
                count1 +=1
                print(count1)
                rotate_left(ep_chassis)
            else:
                # ep_gimbal.recenter(yaw_speed=200).wait_for_completed()
                # time.sleep(0.1)
                rotate_180_degrees(ep_chassis)
       

        ep_gimbal.recenter(yaw_speed=200).wait_for_completed()
        time.sleep(0.1)
        adjust_angle(yaw)
        print('yaw = ', yaw)
        time.sleep(0.1)

        if tof_data and tof_data[-1] >= 6000 and current_left >= 49 and current_left >= 49: 
            print(tof_data[-1], current_right, current_left)
            break

    ep_chassis.unsub_position()
    ep_chassis.unsub_attitude()
    ep_sensor.unsub_distance()
    ep_sensor_adaptor.unsub_adapter()
    ep_robot.close()

    # # Save data to CSV
    # with open('chassis_data.csv', 'w', newline='') as csvfile:
    #     fieldnames = ['timestamp', 'position_x', 'position_y', 'position_z', 'tof_distance', 'left_data', 'right_data']
    #     writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    #     writer.writeheader()
    #     for idx in range(max(len(tof_time_data), len(left_time_data), len(right_time_data))):
    #         row = {'position_x': position_data[idx][0] if idx < len(position_data) else '',
    #                'position_y': position_data[idx][1] if idx < len(position_data) else '',
    #                'position_z': position_data[idx][2] if idx < len(position_data) else '',
    #                'tof_distance': tof_data[idx] if idx < len(tof_data) else '',
    #                'left_data': left_data[idx] if idx < len(left_data) else '',
    #                'right_data': right_data[idx] if idx < len(right_data) else ''}
    #         writer.writerow(row)

    # # Plotting data
    # plt.figure()


    # plt.subplot(2, 2, 2)
    # if tof_time_data:
    #     time_series = [t - tof_time_data[0] for t in tof_time_data]
    #     plt.plot(time_series, tof_data, label='Distance')
    # plt.title("TOF Data")
    # plt.legend()

    # plt.subplot(2, 2, 3)
    # if left_time_data:
    #     time_series = [t - left_time_data[0] for t in left_time_data]
    #     plt.plot(time_series, left_data, label='Left Sensor')
    #     plt.plot(time_series, right_data, label='Right Sensor')
    # plt.title("Left/Right Sensor Data")
    # plt.legend()

    # plt.tight_layout()
    # plt.show()



