import time
import numpy as np
from robomaster import robot

# ตัวแปร global
current_x = 0.0
current_y = 0.0
tof_data = []  # ข้อมูลจากเซ็นเซอร์ ToF
left_data = []  # ข้อมูลจากเซ็นเซอร์ด้านซ้าย
right_data = []  # ข้อมูลจากเซ็นเซอร์ด้านขวา
current_left = 0.0
current_right = 0.0
has_large_gap = True  # ตัวบ่งชี้ว่ามีช่องว่างขนาดใหญ่หรือไม่
gap_count = 0  # จำนวนครั้งที่พบช่องว่างขนาดใหญ่

def update_position(position_info):
    global current_x, current_y
    # อัปเดตตำแหน่งปัจจุบันของหุ่นยนต์
    current_x, current_y, z = position_info

def update_tof(tof_info):
    # อัปเดตข้อมูลจากเซ็นเซอร์ ToF
    tof_data.append(tof_info[0])

def update_sensor_data(sub_info):
    global current_left, current_right
    io_data, ad_data = sub_info
    
    distances = []
    for adc_value in ad_data:
        voltage = adc_value * 3.3 / 1023

        # คำนวณระยะทางจากค่าแรงดัน
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

    # คำนวณระยะทางเฉลี่ยของเซ็นเซอร์ซ้ายและขวา
    left = sum(distances[0:2]) / 2
    right = sum(distances[2:4]) / 2

    current_left = left
    current_right = right

    if 9 <= left <= 13:
        left += 1
    elif 14 <= left <= 16:  
        left += 3

    if left >= 12.9:
        left = 50
    if right >= 25:
        right = 50

    print("ad_data", ad_data)
    print(f"port1 left: {left}, port2 right: {right}")

def move_until_threshold(ep_chassis, threshold_distance):
    global has_large_gap, gap_count

    while True:
        # หยุดหากเซ็นเซอร์ ToF แสดงค่าต่ำกว่าค่าที่กำหนด
        if tof_data and tof_data[-1] < threshold_distance:
            print(f'TOF < threshold_distance : {tof_data[-1]}')
            break

        # หมุนหุ่นยนต์หากเซ็นเซอร์ด้านขวาต่ำกว่าค่าที่กำหนด
        if current_right <= 10:
            turn_right(ep_chassis)
            print(f"turn_right")
        # หมุนหุ่นยนต์หากเซ็นเซอร์ด้านซ้ายต่ำกว่าค่าที่กำหนด
        elif current_left <= 10:
            turn_left(ep_chassis)
            print(f"turn_left")
        # หยุดหากพบช่องว่างขนาดใหญ่และเซ็นเซอร์ด้านขวาเกินค่าที่กำหนด
        elif has_large_gap and current_right >= 49:
            stop_robot(ep_chassis)
            print(f"stop_robot")
            break
        else:
            move_forward(ep_chassis)
            print("move_forward")

        # ปรับสถานะของช่องว่างขนาดใหญ่
        if gap_count == 15:
            has_large_gap = True
            gap_count = 0
        if gap_count >= 1:
            gap_count += 1

        time.sleep(0.1)

    stop_robot(ep_chassis)
    time.sleep(0.5)

def move_forward(ep_chassis):
    # เคลื่อนที่ไปข้างหน้า
    ep_chassis.drive_wheels(w1=55, w2=55, w3=55, w4=55)

def stop_robot(ep_chassis):
    # หยุดหุ่นยนต์
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

def turn_right(ep_chassis):
    # หมุนหุ่นยนต์ไปทางขวา
    ep_chassis.drive_wheels(w1=17, w2=-17, w3=17, w4=-17)

def turn_left(ep_chassis):
    # หมุนหุ่นยนต์ไปทางซ้าย
    ep_chassis.drive_wheels(w1=-17, w2=17, w3=-17, w4=17)

def rotate_180_degrees(ep_chassis):  
    # หมุนหุ่นยนต์ 180 องศา
    ep_chassis.move(x=0, y=0, z=180, xy_speed=15).wait_for_completed()
    time.sleep(0.5)

def rotate_left(ep_chassis):  
    # หมุนหุ่นยนต์ไปทางซ้าย 90 องศา
    ep_chassis.move(x=0, y=0, z=90, xy_speed=15).wait_for_completed()
    time.sleep(0.5)

def rotate_right(ep_chassis):  
    # หมุนหุ่นยนต์ไปทางขวา 90 องศา
    ep_chassis.move(x=0, y=0, z=-90, xy_speed=15).wait_for_completed()
    time.sleep(0.5)

def scan_and_rotate(ep_chassis, ep_gimbal):
    global has_large_gap, gap_count

    # สแกนหาด้วย gimbal
    ep_gimbal.moveto(pitch=0, yaw=90, pitch_speed=0, yaw_speed=100).wait_for_completed()
    time.sleep(0.2)

    # หมุนหุ่นยนต์ตามการวิเคราะห์ข้อมูล ToF
    if tof_data and tof_data[-1] >= 700:
        has_large_gap = False
        gap_count += 1
        rotate_right(ep_chassis)
    else:
        ep_gimbal.recenter().wait_for_completed()
        time.sleep(0.2)
        ep_gimbal.moveto(pitch=0, yaw=-90, pitch_speed=0, yaw_speed=40).wait_for_completed()
        time.sleep(0.2)
        if tof_data and tof_data[-1] >= 700:
            has_large_gap = False
            gap_count += 1
            rotate_left(ep_chassis)
        else:
            ep_gimbal.recenter().wait_for_completed()
            time.sleep(0.2)
            rotate_180_degrees(ep_chassis)

    ep_gimbal.recenter().wait_for_completed()
    time.sleep(0.2)

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor
    ep_sensor_adaptor = ep_robot.sensor_adaptor
    ep_gimbal = ep_robot.gimbal

    # callback สำหรับข้อมูลตำแหน่งและเซ็นเซอร์
    ep_chassis.sub_position(freq=10, callback=update_position)
    ep_sensor.sub_distance(freq=10, callback=update_tof)
    ep_sensor_adaptor.sub_adapter(freq=10, callback=update_sensor_data)  # สมัครสมาชิกข้อมูลอนาล็อก
    time.sleep(1)

    while True:
        move_until_threshold(ep_chassis, 350)  # เคลื่อนที่จนกว่าจะพบเซ็นเซอร์ ToF น้อยกว่าค่าที่กำหนด

        scan_and_rotate(ep_chassis, ep_gimbal)  # สแกนและหมุนหุ่นยนต์

        # หยุดหากเซ็นเซอร์ ToF มีค่ามากกว่า 5000 และเซ็นเซอร์ด้านซ้ายและขวามีค่ามากกว่า 49
        if tof_data and tof_data[-1] >= 5000 and current_left >= 49 and current_right >= 49: 
            print(tof_data[-1], current_right, current_left)
            break

    # ปิดการเชื่อมต่อหุ่นยนต์
    ep_chassis.unsub_position()
    ep_sensor.unsub_distance()
    ep_sensor_adaptor.unsub_adapter()
    ep_robot.close()
