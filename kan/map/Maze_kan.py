import time
from robomaster import robot
import math

# map
maze_size = 4
visited = [[0 for _ in range(maze_size)] for _ in range(maze_size)]

# กำหนดการเคลื่อนที่ในทิศทาง: ขึ้น, ขวา, ลง, ซ้าย (ตามลำดับ)
directions = [(0, 1), (1, 0), (0, -1), (-1, 0)] # x, y
current_position = (0, 0)  # ตำแหน่งเริ่มต้น (0, 0)
yaw_angles = [0, -90, 180, 90]  # ทิศทางที่ตรงกับ yaw

def is_valid_move(x, y): # เช็คว่าตำแหน่ง (x, y) อยู่ในเขตของเขาวงกตและยังไม่ได้สำรวจ
    return 0 <= x < maze_size and 0 <= y < maze_size and visited[x][y] == 0


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
    tof_data.append(tof_info[0])
    # print('----------------------------------')
    # print(f'TOF: {tof_info[0]}')
    # print('----------------------------------')

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
    # print('----------------------------------')
    # print(f'ir_left: {ir_left}, ir_right: {ir_right}')
    # print(f"PORT1 left: {sharp_left}, PORT2 right: {sharp_right}")
    # print('----------------------------------')
    return distances

def move_forward(ep_chassis):
    ep_chassis.drive_wheels(w1=50, w2=50, w3=50, w4=50)

def stop_move(ep_chassis):
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  

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

def scan_gimbal(ep_gimbal):
    angles_to_scan = [-90, 0, 90]  # Scan left, center, right
    for angle in angles_to_scan:
        ep_gimbal.moveto(pitch=0, yaw=angle, yaw_speed=200).wait_for_completed()
        time.sleep(1)  # Allow time for sensor reading
        if len(tof_data) > 0 and tof_data[-1] > 300:  # Clear path found
            return angle
    return None  # No clear path found

def check_obstacle():
    if tof_data and len(tof_data) > 0 and tof_data[-1] <= 330:
        return True
    return False


if __name__ == '__main__':
    # เริ่มต้นการเชื่อมต่อกับหุ่นยนต์
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor
    ep_sensor_adaptor = ep_robot.sensor_adaptor
    ep_gimbal = ep_robot.gimbal

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
    
    threshold_distance = 330

    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)
    ep_sensor.sub_distance(freq=10, callback=sub_tof_handler)
    ep_sensor_adaptor.sub_adapter(freq=10, callback=sub_data_handler)  
    time.sleep(0.5)

    # รีเซ็ตตำแหน่ง gimbal ให้อยู่ในตำแหน่งกลาง
    ep_gimbal.recenter().wait_for_completed()
    time.sleep(0.1)

    # explor_maze
    while True:
        current_time = time.time()
        if check_obstacle():
            stop_move(ep_chassis)
            
            clear_path_angle = scan_gimbal(ep_gimbal)
            if clear_path_angle is not None:
                print(f"Clear path found at {clear_path_angle} degrees!")
                if clear_path_angle == 90:
                        rotate_right(ep_chassis)
                elif clear_path_angle == -90:
                        rotate_left(ep_chassis)
                    
                move_forward(ep_chassis)
            else:
                    print("No clear path found, turning around!")
                    rotate_180_degrees(ep_chassis)
        else:
            move_forward(ep_chassis)
            

        

        # ยกเลิกการสมัครสมาชิกเซ็นเซอร์และปิดการเชื่อมต่อหุ่นยนต์
        ep_chassis.unsub_position()
        ep_chassis.unsub_attitude()
        ep_sensor.unsub_distance()
        ep_sensor_adaptor.unsub_adapter()
        ep_robot.close()
