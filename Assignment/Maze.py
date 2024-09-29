import time
# import matplotlib.pyplot as plt
from robomaster import robot

# กำหนดlist เพื่อเก็บข้อมูล เเละกำหนดตัวเเปรค่าเริ่มต้น
position_data = []
tof_data = []
left_data = []
right_data = []
current_x = 0.0
current_y = 0.0
current_left = 0.0
current_right = 0.0
ir_left = 0
ir_right = 0
count = 0
yaw = None
status = True

# Function Callback สำหรับจัดการข้อมูลตำแหน่งจากเซ็นเซอร์
def sub_position_handler(position_info):
    x, y, z = position_info
    position_data.append((round(x,2), (round(y,2)), (round(z,2))))
    # print("chassis position: x:{:.2f}, y:{:.2f}, z:{:.2f}".format(x, y, z))

# Function Callback สำหรับจัดการข้อมูลท่าทาง (yaw, pitch, roll) ของหุ่นยนต์
def sub_attitude_info_handler(attitude_info):
    global yaw
    yaw, pitch, roll = attitude_info
    # print("chassis attitude: yaw:{0}, pitch:{1}, roll:{2} ".format(yaw, pitch, roll))

# Function Callback สำหรับจัดการข้อมูลจากเซ็นเซอร์ TOF
def sub_tof_handler(tof_info):
    time.sleep(0.1)
    tof_data.append(tof_info[0])
    # print(f'TOF: {tof_info[0]}')

# Function กรองข้อมูลจาก ADC เพื่อให้ค่าเสถียรมากขึ้น
def filter_adc_value(ad_data):
    y_filtered = []
    alpha = 0.1  # ค่าอัลฟาสำหรับการกรอง
    y_prev = 0  # กำหนดค่าเริ่มต้นของ y_prev

    for adc_value in ad_data: 
        y_current = alpha * y_prev + (1 - alpha) * adc_value
        y_filtered.append(y_current)
        y_prev = y_current

    return y_filtered 

# Function Callback สำหรับจัดการข้อมูลจากเซ็นเซอร์ Sharp
def sub_data_handler(sub_info):
    global current_left, current_right, ir_left, ir_right
    io_data, ad_data = sub_info
    
    y_filtered = filter_adc_value(ad_data)

    distances = []
    for adc_filtered in  y_filtered:
        voltage = adc_filtered * 3.3 / 1023

        # การคำนวณระยะทางจากแรงดันไฟฟ้า
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
    
    # การคำนวณค่าเฉลี่ยของเซ็นเซอร์Sharpซ้ายและขวา
    sharp_left = sum(distances[0:2]) / 2
    sharp_right = sum(distances[2:4]) / 2
    left_data.append(sharp_left)
    right_data.append(sharp_right)
    
    # ปรับค่า sharp_left และ sharp_right ตามเกณฑ์ที่กำหนด
    # if sharp_left >= 13:
    #     sharp_left += 2
    
    if sharp_left >= 20:
        sharp_left = 50
    if sharp_right >= 26:
        sharp_right = 50

    current_right = sharp_right
    current_left = sharp_left

    ir_left = ep_sensor_adaptor.get_io(id=1, port=2)
    ir_right = ep_sensor_adaptor.get_io(id=2, port=1)

    ir_left = io_data[1]
    ir_right = io_data[2]
    print(f'ir_left: {ir_left}, ir_right: {ir_right}')
    print(f"PORT1 left: {sharp_left}, PORT2 right: {sharp_right}")
    return distances

# Function เคลื่อนที่ไปข้างหน้าจนกว่าจะเจอกำเเพงด้านหน้า หรือเจอทางทางขวาที่ไปได้
def move_forword(ep_chassis, threshold_distance, overall_start_time, time_data, list_current_x):
    global current_left, current_right, count, status 

    while True:
        # # แสดงผลเส้นทางการเคลื่อนที่ของหุ่นยนต์แบบเรียลไทม์
        # if position_data:
        #     x_vals = [pos[0] for pos in position_data]
        #     y_vals = [pos[1] for pos in position_data]
        #     ax.clear()
        #     ax.plot(y_vals, x_vals ,'*-', label="Robot Path")
        #     ax.set_xlabel('Y Position (cm)')
        #     ax.set_ylabel('X Position (cm)')
        #     ax.set_title('Real-time Robot Path')
        #     ax.grid(True)
        #     plt.draw()
        #     plt.pause(0.01) 
        
        # หยุดหุ่นยนต์หาก TOF น้อยกว่าค่า threshold (เจอกำเเพงด้านหน้า)
        # if tof_data and tof_data[-1] < threshold_distance or (ir_left == 0 and ir_left ==0):
        if tof_data and tof_data[-1] < threshold_distance :
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
            time.sleep(0.05)
            count = 0
            # print(tof_data[-1],ir_left,ir_right)
            print('----------------------------------')
            break

        # หยุดหุ่นยนต์หากค่า current_right มากกว่า 49 (ทางขวาไปได้)
        elif status and current_right >= 49:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0) 
            count = 0
            time.sleep(0.05)
            print('************************************')
            break

        # current_right หรือ current_left ต่ำกว่า 10 ให้ขยับรถเข้ากลาง เพื่อไม่ให้ชนกำเเพง
        elif current_right <= 10 :#or (ir_right == 0 and ir_left == 1) :
            ep_chassis.drive_wheels(w1=18, w2=-18, w3=18, w4=-18)  
            print('<')
        elif current_left <= 10 :#or (ir_left == 0 and ir_right == 1):
            ep_chassis.drive_wheels(w1=-18, w2=18, w3=-18, w4=18)  
            print('>')


        # เคลื่อนที่ไปด้านหน้า
        else:
            ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
            print('speed =', speed) 

        # ปรับค่า count และ status
        if count == 25:
            status = True
            count = 0

        if count >= 1:
            count += 1
        # print(count)
        # print(status)

        list_current_x.append((current_x, current_y))
        time_data.append(time.time() - overall_start_time)
        
        time.sleep(0.1)  

    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    time.sleep(0.1)

# Functionหมุนหุ่นยนต์ 180 องศา
def rotate_180_degrees(ep_chassis):  
    ep_chassis.move(x=0, y=0, z=180, z_speed=80).wait_for_completed()
    time.sleep(0.1)

# Functionหมุนหุ่นยนต์ไปทางซ้าย 90 องศา
def rotate_left(ep_chassis):  
    ep_chassis.move(x=0, y=0, z=90, z_speed=80).wait_for_completed()
    time.sleep(0.1)

# Functionหมุนหุ่นยนต์ไปทางขวา 90 องศา
def rotate_right(ep_chassis):  
    ep_chassis.move(x=0, y=0, z=-90, z_speed=80).wait_for_completed()
    time.sleep(0.1)

# Function ปรับมุม yaw ของหุ่นยนต์ให้ตรงตามที่กำหนด
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
    # เริ่มต้นการเชื่อมต่อกับหุ่นยนต์
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor
    ep_sensor_adaptor = ep_robot.sensor_adaptor
    ep_gimbal = ep_robot.gimbal

    kp, ki, kd = 0.5, 0.002, 0.15   
    tolerance = 0.01   
    prev_error, integral = 0.0, 0.0
    prev_time = time.time()

    # สมัครสมาชิกเพื่อรับข้อมูลจากเซ็นเซอร์ต่าง ๆ
    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)
    ep_sensor.sub_distance(freq=10, callback=sub_tof_handler)
    ep_sensor_adaptor.sub_adapter(freq=10, callback=sub_data_handler)  
    time.sleep(0.5)

    time_data, list_current_x = [], []
    overall_start_time = time.time()

    # รีเซ็ตตำแหน่ง gimbal ให้อยู่ในตำแหน่งกลาง
    ep_gimbal.recenter().wait_for_completed()
    time.sleep(0.05)

    # เริ่มplotเส้นทาง
    # plt.ion()  
    # fig, ax = plt.subplots()

    while True:
 
        current_time = time.time()
        error = (tof_data[-1] - 300) / 10
        time_diff = current_time - prev_time
        integral += error * time_diff
        derivative = (error - prev_error) / time_diff if time_diff > 0 else 0.0
        speed = kp * error + kd * derivative + ki * integral 
        speed = max(min(speed, 70), 0)  
        # เคลื่อนที่ไปข้างหน้าจนกว่า TOF จะน้อยกว่า 300 หรือเจอทางทางขวาที่ไปได้
        move_forword(ep_chassis, 300, overall_start_time, time_data, list_current_x)  
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
        time.sleep(0.1)

        while True:
            # เคลื่อนที่ถอยหลังหาก TOF น้อยกว่า 160
            if tof_data[-1] < 160:
                ep_chassis.drive_wheels(w1=-20, w2=-20, w3=-20, w4=-20) 
                print('backword')

            # เคลื่อนที่ไปข้างหน้าหาก TOF อยู่ในช่วง 310-410
            elif tof_data[-1] > 310 and tof_data[-1] < 410:
                ep_chassis.drive_wheels(w1=20, w2=20, w3=20, w4=20) 
                print('forword')
            
            # เคลื่อนที่ไปทางซ้าย หาก current_right น้อยกว่า 10

            # เคลื่อนที่ไปทางขวา หาก current_left น้อยกว่า 10
            elif current_left <= 10 :#or (ir_left == 0 and ir_right == 1):
                ep_chassis.drive_wheels(w1=-18, w2=18, w3=-18, w4=18)                   
                # print(current_left,ir_left)
                print('>>')

            elif current_right <= 10 :#or (ir_right == 0 and ir_left == 1):
                ep_chassis.drive_wheels(w1=18, w2=-18, w3=18, w4=-18) 
                # print(current_right,ir_right)
                print('<<')
            else:
                # หยุดนิ่ง
                ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  
                break 
            
        prev_error = error
        prev_time = current_time  
        time.sleep(0.1)

        # หมุนหุ่นยนต์ไปทางขวา หากทางขวาไปได้
        if current_right >= 49:
                status = False
                count +=1
                # print(current_right)
                # print('turn_right',status)
                rotate_right(ep_chassis)

        else:
            # หมุนหุ่นยนต์ไปทางซ้าย หากทางซ้ายไปได้
            if current_left >= 49:
                status = False
                count +=1
                # print(current_right,current_left)
                # print('turn_left',status)
                rotate_left(ep_chassis)
            else:
                # หมุนหุ่นยนย์กลับหลัง เพราะเจอทางตัน
                count += 1
                # print(current_right,current_left)
                # print('turn_back')
                rotate_180_degrees(ep_chassis)
                
       
        # รีเซ็ตตำแหน่ง gimbal
        ep_gimbal.recenter(yaw_speed=200).wait_for_completed()
        time.sleep(0.1)
        
        # ปรับมุม yaw ให้ตรง
        adjust_angle(yaw)
        time.sleep(0.2)

        # หยุดการทำงานหาก TOF มากกว่า 6000 และ current_left, current_right มากกว่า 49
        if tof_data and tof_data[-1] >= 6000 and current_left >= 49 and current_left >= 49: 
            break

    # ยกเลิกการสมัครสมาชิกเซ็นเซอร์และปิดการเชื่อมต่อหุ่นยนต์
    ep_chassis.unsub_position()
    ep_chassis.unsub_attitude()
    ep_sensor.unsub_distance()
    ep_sensor_adaptor.unsub_adapter()
    ep_robot.close()
