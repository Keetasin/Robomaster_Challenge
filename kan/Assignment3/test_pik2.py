import robomaster
from robomaster import robot, blaster, camera
import math
import matplotlib.pyplot as plt
import time
import cv2
import numpy as np

from animate_text import *

MAX_SPEED = 8

#[North(front), West(left), East(right), South(back),Visited]
grid = [
        [[2,2,0,0,0],[2,0,0,0,0],[2,0,0,0,0],[2,0,2,0,0]],
        [[0,2,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,2,0,0]],
        [[0,2,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,2,0,0]],
        [[0,2,0,2,0],[0,0,0,2,0],[0,0,0,2,0],[0,0,2,2,0]]
        ]

# start map
row = 0
col = 0
cerrent_position = [row, col]
TOF_front_status = None
TOF_distance = 0

''' ----- image ----- '''
class Marker:
    def __init__(self, x, y, w, h):
        self._x = x
        self._y = y
        self._w = w
        self._h = h

    @property
    def pt1(self):
        return int(self._x), int(self._y)

    @property
    def pt2(self):
        return int(self._x + self._w), int(self._y + self._h)

    @property
    def center(self):
        return int(self._x + self._w / 2), int(self._y + self._h / 2)
    

def process_image(image_path, output_path, alpha=1.5, beta=0, canny_threshold1=40, canny_threshold2=160, dilation_kernel_size=(3, 3), dilation_iterations=1):
    image = cv2.imread(image_path)[250:480,570:720] 
    b, g, r = cv2.split(image)    
    b_contrasted = cv2.convertScaleAbs(b, alpha=alpha, beta=beta) 
    edges_b = cv2.Canny(b_contrasted, canny_threshold1, canny_threshold2)
    kernel = np.ones(dilation_kernel_size, np.uint8)
    edges_dilated = cv2.dilate(edges_b, kernel, iterations=dilation_iterations)
    cv2.imwrite(output_path, edges_dilated)


def detect_blue_circles(frame, count):

    """
    ตรวจจับวงกลมสีฟ้าในเฟรมและวาดกรอบสี่เหลี่ยมรอบวงกลมที่ตรวจจับได้

    Parameters:
    - frame: ภาพจากกล้องในรูปแบบ BGR

    Returns:
    - Marker: ตำแหน่งและขนาดของวงกลมที่ตรวจจับได้ (ถ้ามี)
    """
    output_image = frame.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([100, 150, 0])
    upper_blue = np.array([140, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    result = cv2.bitwise_and(frame, frame, mask=mask)
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    circles = cv2.HoughCircles(
        gray_blurred, 
        cv2.HOUGH_GRADIENT, 
        dp=1, 
        minDist=50, 
        param1=50, 
        param2=25, 
        minRadius=10, 
        maxRadius=100
    )

    marker = None

    if circles is not None and count < 12:
        print('พบวงกลม')
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            x, y, r = i[0], i[1], i[2]

            roi = mask[y - r:y + r, x - r:x + r]
            if roi.size == 0:
                continue

            contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                perimeter = cv2.arcLength(contour, True)
                area = cv2.contourArea(contour)
                
                if perimeter == 0:
                    continue

                circularity = 4 * np.pi * (area / (perimeter * perimeter))
                
                if circularity > 0.6 :
                    cv2.circle(output_image, (x, y), r, (0, 0, 255), 2)
                    cv2.circle(output_image, (x, y), 2, (0, 0, 255), 3)

                    top_left = (x - r, y - r)
                    bottom_right = (x + r, y + r)

                    top_left = (max(top_left[0], 0), max(top_left[1], 0))
                    bottom_right = (min(bottom_right[0], frame.shape[1] - 1), min(bottom_right[1], frame.shape[0] - 1))

                    cv2.rectangle(output_image, top_left, bottom_right, (0, 0, 255), 2)  # สีฟ้า

                    w = bottom_right[0] - top_left[0]
                    h = bottom_right[1] - top_left[1]

                    x,y,w,h = top_left[0]-15, top_left[1]-5, w+27, h+80 
                    cv2.rectangle(output_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(output_image, f" x: {x}, y: {y}, w: {w}, h: {h}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

                    marker = Marker(x, y, w, h)
                    break 
            if marker:
                break
    
    center_x, center_y = output_image.shape[1] // 2, output_image.shape[0] // 2
    cv2.line(output_image, (center_x - 10, center_y), (center_x + 10, center_y), (0, 0, 255), 2)  # แนวนอน
    cv2.line(output_image, (center_x, center_y - 10), (center_x, center_y + 10), (0, 0, 255), 2)  # แนวตั้ง

    cv2.imshow('Detected Blue Circles with Rectangles', output_image)
    cv2.waitKey(1)  

    return marker


def matches_template():
    img1 = cv2.imread('template.png')
    img2 = cv2.imread('detect.png')

    orb = cv2.ORB_create()

    keypoints1, descriptors1 = orb.detectAndCompute(img1, None)
    keypoints2, descriptors2 = orb.detectAndCompute(img2, None)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(descriptors1, descriptors2)
    matches = sorted(matches, key=lambda x: x.distance)
    similarity_score = len([m for m in matches if m.distance < 50])  
    print(f"คะแนนความคล้ายของรูปภาพคือ: {similarity_score}")

    if similarity_score >= 5: ep_blaster.fire(fire_type=blaster.INFRARED_FIRE, times=1)


def detect_chicken(frame):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_chicken = np.array([33, 150, 100])
    upper_chicken = np.array([38, 255, 255])

    mask_chicken = cv2.inRange(hsv_frame, lower_chicken, upper_chicken)
    contours_chicken, _ = cv2.findContours(mask_chicken, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours_chicken: 
        max_chicken_contour = max(contours_chicken, key=cv2.contourArea)
        (x_chicken, y_chicken, w_chicken, h_chicken) = cv2.boundingRect(max_chicken_contour)
        
        if 60 <= w_chicken >= 50 or (w_chicken > 30 and 1.1 <= (h_chicken/w_chicken) >= 1.7):
            if w_chicken > 65:
                add_w_chicken = int(w_chicken * 0.3)
                add_h_chicken = int(h_chicken * 0.6)
                new_y_chicken = y_chicken - add_h_chicken // 2 + 20

            else :
                add_w_chicken = int(w_chicken * 0.35)
                add_h_chicken = int(h_chicken * 0.55)
                new_y_chicken = (y_chicken - add_h_chicken // 2 + 20) - 15

            new_x_chicken = x_chicken - add_w_chicken // 2
            new_w_chicken = w_chicken + add_w_chicken
            new_h_chicken = h_chicken + add_h_chicken

            cv2.rectangle(frame, (new_x_chicken, new_y_chicken), (new_x_chicken + new_w_chicken, new_y_chicken + new_h_chicken), (255, 0, 255), 2)
            cv2.putText(frame, f" chicken x: {x_chicken}, y: {y_chicken}, w: {w_chicken}, h: {h_chicken}", (x_chicken, y_chicken - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            print('chickkkkkkkkkkkk')

def detect():
    global status_logic, count, check, p, frame, marker, accumulate_err_x ,accumulate_err_y, data_pitch_yaw, prev_time 

    for _ in range (2):
        time.sleep(1)
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=1)
    marker = detect_blue_circles(frame, count)
    detect_chicken(frame)
    if marker and marker._w >= 30 and marker._w < 100 and count == 0: check = False
    if count > 0:count += 1

    print('--->',check)

    if check == False:
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        time.sleep(0.05)
        if marker:
            after_time = time.time()
            x, y = marker.center

            err_x = center_x - x
            err_y = center_y - y
            accumulate_err_x += err_x * (after_time - prev_time)
            accumulate_err_y += err_y * (after_time - prev_time)

            speed_x = p * err_x
            speed_y = p * err_y
            ep_gimbal.drive_speed(pitch_speed=speed_y, yaw_speed=-speed_x)

            data_pitch_yaw.append([err_x, err_y, round(speed_x, 3), round(speed_y, 3), after_time - prev_time])

            prev_time = after_time

            if marker._w >= 45:
                count += 1
                if count >= 5 and count < 8:
                    ep_blaster.set_led(brightness=255, effect=blaster.LED_ON)
                    time.sleep(0.2)

        if count == 5:
            temp_image_path = 'temp_frame.png'
            cv2.imwrite(temp_image_path, frame)
            process_image(
                image_path=temp_image_path,
                output_path='detect.png',
                alpha=1.5,
                beta=0,
                canny_threshold1=40,
                canny_threshold2=160,
                dilation_kernel_size=(3, 3),
                dilation_iterations=1
            )
        if count == 8:
            matches_template()

        if count >=12:
            ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
            ep_blaster.set_led(brightness=0, effect=blaster.LED_OFF)
            check = True
            count = 0
            print('---------')
        else:
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)



''' ----- TOF Sensor ----- '''
def tof_data_handler(sub_info):
    global TOF_distance, TOF_front_status
    TOF_distance = sub_info[0]
    # print(f"ToF distance: {tof_distance} mm")

    sharp_sensor_data()


''' ----- Sharp Sensor ----- '''
status_sharp_left = None
adc_left = None
adc_l_new = 0

status_sharp_right = None
adc_right = None
adc_r_new = 0

def sharp_sensor_data():
    global adc_right, adc_left, adc_l_new, adc_r_new, status_sharp_right, status_sharp_left

    adc_right = ep_sensor_adaptor.get_adc(id=2, port=2)
    adc_left = ep_sensor_adaptor.get_adc(id=1, port=1)

    voltage_r = adc_right * 3.3 / 1023
    voltage_l = adc_left * 3.3 / 1023

    # คำนวณระยะทางจากแรงดันไฟฟ้าเซ็นเซอร์ด้านขวา
    if 2.2 <= voltage_r < 3.2:
        adc_r_new = (voltage_r - 4.30764) / -0.3846
    elif 1.4 <= voltage_r < 2.2:
        adc_r_new = (voltage_r - 3.2) / -0.2
    elif 0.8 <= voltage_r < 1.4:
        adc_r_new = (voltage_r - 1.87) / -0.067
    elif 0.4 <= voltage_r < 0.8:
        adc_r_new = (voltage_r - 1.344) / -0.034
    else:
        if voltage_r >= 3.2:
            adc_r_new = (voltage_r - 4.30764) / -0.3846
        elif voltage_r < 0.4:
            adc_r_new = (voltage_r - 1.344) / -0.034

    # คำนวณระยะทางจากแรงดันไฟฟ้าเซ็นเซอร์ด้านซ้าย
    if 2.2 <= voltage_l < 3.2:
        adc_l_new = (voltage_l - 4.30764) / -0.3846
    elif 1.4 <= voltage_l < 2.2:
        adc_l_new = (voltage_l - 3.2) / -0.2
    elif 0.8 <= voltage_l < 1.4:
        adc_l_new = (voltage_l - 1.87) / -0.067
    elif 0.4 <= voltage_l < 0.8:
        adc_l_new = (voltage_l - 1.344) / -0.034
    else:
        if voltage_l >= 3.2:
            adc_l_new = (voltage_l - 4.30764) / -0.3846
        elif voltage_l < 0.4:
            adc_l_new = (voltage_l - 1.344) / -0.034

    if adc_r_new <= 27:
        status_sharp_right = True
    else:
        status_sharp_right = False

    if adc_l_new <= 24:
        status_sharp_left = True
    else:
        status_sharp_left = False

    # print(f"distance from front wall: right {adc_right} left {adc_left}")
    # print(f"distance from front wall: left {adc_l_new} right {adc_r_new}")

''' ----- sub_position_handler ----- '''
def sub_position_handler(position_info):
    global x,y
    x, y, z = position_info


''' ----- sub_attitude_info_handler ----- '''
yaw = 0
threshold_sharp = 30

def sub_attitude_info_handler(attitude_info):
    global yaw
    yaw, pitch, roll = attitude_info
    # print(f"chassis attitude: yaw:{yaw}")


''' ----- adjust_all_walls ----- '''
def adjust_all_walls():
    adjust_front_wall()
    adjust_left_wall()
    adjust_right_wall()

def adjust_front_wall():
    if TOF_distance < 120:
        adjustment_x = (abs(TOF_distance - 120)/1000)+0.02
        ep_chassis.move(x=-adjustment_x, y=0, z=0, xy_speed=MAX_SPEED).wait_for_completed()

    if 300 < TOF_distance <= 450:
        adjustment_x = (abs(TOF_distance - 300)/1000)+0.02
        ep_chassis.move(x=adjustment_x, y=0, z=0, xy_speed=MAX_SPEED).wait_for_completed()

def adjust_left_wall():
    if adc_l_new < threshold_sharp:
        if adc_l_new < 15:
            adjustment_y = (abs(25 - adc_l_new) / 100) - 0.03
            ep_chassis.move(x=0, y=adjustment_y, z=0, xy_speed=MAX_SPEED).wait_for_completed()

        elif adc_l_new > 25:
            adjustment_y = (abs(35 - adc_l_new) / 100) - 0.03
            ep_chassis.move(x=0, y=-adjustment_y, z=0, xy_speed=MAX_SPEED).wait_for_completed()

def adjust_right_wall():
    if adc_r_new < threshold_sharp:
        if adc_r_new < 15:
            adjustment_y = (abs(25 - adc_l_new) / 100) - 0.03
            ep_chassis.move(x=0, y=-adjustment_y, z=0, xy_speed=MAX_SPEED).wait_for_completed()

        elif adc_r_new > 25:
            adjustment_y = (abs(35 - adc_l_new) / 100) - 0.03
            ep_chassis.move(x=0, y=adjustment_y, z=0, xy_speed=MAX_SPEED).wait_for_completed()


''' ----- movement ----- '''
def move_stop():
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.75)
    time.sleep(0.7)

def move_forward():
    # adjust_all_walls()
    print('run')
    ep_chassis.move(x=0.6, y=0, z=0, xy_speed=0.7).wait_for_completed()
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
    move_stop()
    print("Movement: Drive forward")

def turn_back():
    ep_chassis.move(x=0, y=0, z=180, z_speed=80).wait_for_completed()
    time.sleep(0.1)
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
    print("Movement: Turn Back")

def turn_left():
    ep_chassis.move(x=0, y=0, z=90, z_speed=80).wait_for_completed()
    time.sleep(0.1)
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
    print("Movement: Turn Left") 
    

def turn_right():
    ep_chassis.move(x=0, y=0, z=-90, z_speed=80).wait_for_completed()
    time.sleep(0.1)
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
    print('Movement: Turn Right')
    
''' ----- DirectionFacing ----- '''
robot_status_now = None
def getDirectionFacing():
    global robo_status_now
    degrees = yaw
    if -45 <= degrees < 0 or 45>=degrees >= 0:
        robo_status_now = 'N'
    if 45 < degrees <= 135:
        robo_status_now = 'E'
    if 135 < degrees <=180 or -180<= degrees <-135 :
        robo_status_now = 'S'
    if -135 <= degrees < -45:
        robo_status_now = 'W'

# Function ปรับมุม yaw ของหุ่นยนต์ให้ตรงตามที่กำหนด
def adjust_angle(yaw):
    print("adjust_yaw")
    target_yaw = 0
    current_yaw = yaw

    if -135 < yaw <= -45:
        target_yaw = -90
        ep_chassis.move(x=0, y=0, z=current_yaw-target_yaw, z_speed=60).wait_for_completed()  
    elif 45 < yaw < 135:
        target_yaw = 90
        ep_chassis.move(x=0, y=0, z=current_yaw-target_yaw, z_speed=60).wait_for_completed()
    elif -45 < yaw <= 45:
        target_yaw = 0
        ep_chassis.move(x=0, y=0, z=current_yaw, z_speed=60).wait_for_completed()
    elif -180 <= yaw < -135 :
        target_yaw = -180
        ep_chassis.move(x=0, y=0, z=current_yaw-target_yaw, z_speed=60).wait_for_completed() 
    elif 135 < yaw <= 180:
        target_yaw = 180
        ep_chassis.move(x=0, y=0, z=current_yaw-target_yaw, z_speed=60).wait_for_completed() 

''' ----- Update_Maze ----- '''
status_logic = None
def update_wall():
    global cerrent_position, status_logic
    
    getDirectionFacing()
    logic()
    print(robo_status_now,'*')
    if robot_status_now == 'N':
        if status_logic == 'move_forward':
            grid[cerrent_position[0]][cerrent_position[1]][2] = 2 # มีกำแพงด้านขวาที่พิกัด x, y
            grid[cerrent_position[0]-1][cerrent_position[1]][4] = 1 # 1 คือเคยไปช่องนั้นมาแล้ว

            cerrent_position = [cerrent_position[0]-1,cerrent_position[1]]

        if status_logic == 'turn right': #turn right and move forward
            # grid[cerrent_position[0]][cerrent_position[1]][2] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have right wall 
            grid[cerrent_position[0]][cerrent_position[1]+1][4] = 1
            
            cerrent_position = [cerrent_position[0],cerrent_position[1]+1]

        if status_logic == 'turn back':
            grid[cerrent_position[0]][cerrent_position[1]][0] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have front wall
            grid[cerrent_position[0]][cerrent_position[1]][1] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have left wall
            grid[cerrent_position[0]][cerrent_position[1]][2] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have right wall 

            grid[cerrent_position[0]+1][cerrent_position[1]][4] = 1
            # grid[cerrent_position[0]+1][cerrent_position[1]][3] = 1

            cerrent_position = [cerrent_position[0]+1,cerrent_position[1]]

        if status_logic == 'turn left':
            grid[cerrent_position[0]][cerrent_position[1]][0] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have front wall
            grid[cerrent_position[0]][cerrent_position[1]][2] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have right wall 
            grid[cerrent_position[0]][cerrent_position[1]-1][4] = 1
            # grid[cerrent_position[0]+1][cerrent_position[1]][3] = 1 
            cerrent_position = [cerrent_position[0],cerrent_position[1]-1]       

    if robo_status_now =='E':  
        if status_logic == 'move_forward':
            grid[cerrent_position[0]][cerrent_position[1]][0] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have front wall 
            grid[cerrent_position[0]][cerrent_position[1]+1][4] = 1
            # grid[cerrent_position[0]+1][cerrent_position[1]][3] = 1


            cerrent_position = [cerrent_position[0],cerrent_position[1]+1]
    
        if status_logic == 'turn right': #turn right and move forward
            # grid[cerrent_position[0]][cerrent_position[1]][2] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have right wall 
            grid[cerrent_position[0]+1][cerrent_position[1]][4] = 1
            
            cerrent_position = [cerrent_position[0]+1,cerrent_position[1]]
        
        if status_logic == 'turn back':
            grid[cerrent_position[0]][cerrent_position[1]][0] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have front wall
            grid[cerrent_position[0]][cerrent_position[1]][2] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have right wall
            grid[cerrent_position[0]][cerrent_position[1]][3] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have back wall

            grid[cerrent_position[0]][cerrent_position[1]-1][4] = 1
            # grid[cerrent_position[0]+1][cerrent_position[1]][3] = 1

            cerrent_position = [cerrent_position[0],cerrent_position[1]-1]

        if status_logic == 'turn left':
            grid[cerrent_position[0]][cerrent_position[1]][2] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have right wall 
            grid[cerrent_position[0]][cerrent_position[1]][3] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have back wall 
            grid[cerrent_position[0]-1][cerrent_position[1]][4] = 1
            # grid[cerrent_position[0]+1][cerrent_position[1]][3] = 1 
            cerrent_position = [cerrent_position[0]-1,cerrent_position[1]]

    if robo_status_now =='S':
        if status_logic == 'move_forward':
            grid[cerrent_position[0]][cerrent_position[1]][1] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have left wall 
            grid[cerrent_position[0]+1][cerrent_position[1]][4] = 1
            # grid[cerrent_position[0]+1][cerrent_position[1]][3] = 1


            cerrent_position = [cerrent_position[0]+1,cerrent_position[1]]
    
        if status_logic == 'turn right': #turn right and move forward
            # grid[cerrent_position[0]][cerrent_position[1]][2] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have right wall 
            grid[cerrent_position[0]][cerrent_position[1]-1][4] = 1
            
            cerrent_position = [cerrent_position[0],cerrent_position[1]-1]
        
        if status_logic == 'turn back':
            
            grid[cerrent_position[0]][cerrent_position[1]][1] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have left wall
            grid[cerrent_position[0]][cerrent_position[1]][2] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have right wall 
            grid[cerrent_position[0]][cerrent_position[1]][3] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have back wall

            grid[cerrent_position[0]+1][cerrent_position[1]][4] = 1
            # grid[cerrent_position[0]+1][cerrent_position[1]][3] = 1

            cerrent_position = [cerrent_position[0]+1,cerrent_position[1]]

        if status_logic == 'turn left':
            grid[cerrent_position[0]][cerrent_position[1]][1] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have left wall 
            grid[cerrent_position[0]][cerrent_position[1]][3] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have back wall 
            grid[cerrent_position[0]][cerrent_position[1]+1][4] = 1
            # grid[cerrent_position[0]+1][cerrent_position[1]][3] = 1 
            cerrent_position = [cerrent_position[0],cerrent_position[1]+1]      

    if robo_status_now =='W':
        if status_logic == 'move_forward':
            grid[cerrent_position[0]][cerrent_position[1]][0] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have back wall 
            grid[cerrent_position[0]][cerrent_position[1]-1][4] = 1
            # grid[cerrent_position[0]+1][cerrent_position[1]][3] = 1


            cerrent_position = [cerrent_position[0],cerrent_position[1]-1]
    
        if status_logic == 'turn right': #turn right and move forward
            # grid[cerrent_position[0]][cerrent_position[1]][2] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have right wall 
            grid[cerrent_position[0]-1][cerrent_position[1]][4] = 1
            
            cerrent_position = [cerrent_position[0]-1,cerrent_position[1]]
        
        if status_logic == 'turn back':
            grid[cerrent_position[0]][cerrent_position[1]][0] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have front wall
            grid[cerrent_position[0]][cerrent_position[1]][1] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have left wall
            grid[cerrent_position[0]][cerrent_position[1]][3] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have back wall

            grid[cerrent_position[0]][cerrent_position[1]+1][4] = 1
            # grid[cerrent_position[0]+1][cerrent_position[1]][3] = 1

            cerrent_position = [cerrent_position[0],cerrent_position[1]+1]

        if status_logic == 'turn left':
            grid[cerrent_position[0]][cerrent_position[1]][1] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have left wall 
            grid[cerrent_position[0]][cerrent_position[1]][0] = 2 #grid[cerrent_position[0]][cerrent_position[1]] have front wall 
            grid[cerrent_position[0]+1][cerrent_position[1]][4] = 1
            # grid[cerrent_position[0]+1][cerrent_position[1]][3] = 1 
            cerrent_position = [cerrent_position[0]+1,cerrent_position[1]]

''' ----- scan gimbal ----- '''

threshold_TOF = 340  # เกณฑ์ที่ใช้ในการตรวจสอบผนัง

# ฟังก์ชันตรวจสอบทุกด้าน
def scan_gimbal_all():
    print('scan_gimbal_all')
    scan_gimbal_left()
    scan_gimbal_front()
    scan_gimbal_right()
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()

# ฟังก์ชันตรวจสอบด้านหน้า
def scan_gimbal_front():
    print('front')
    ep_gimbal.moveto(pitch=0, yaw=0, yaw_speed=300).wait_for_completed()
    time.sleep(1)
    # ตรวจสอบค่า TOF ที่ด้านหน้า
    if TOF_distance <= threshold_TOF:
        return True  # มีกำแพงด้านหน้า
    else:
        return False  # ไม่มีผนังด้านหน้า

# ฟังก์ชันตรวจสอบด้านซ้าย
def scan_gimbal_left():
    print('left')
    ep_gimbal.moveto(pitch=0, yaw=-90, yaw_speed=300).wait_for_completed()
    time.sleep(1)
    # ตรวจสอบค่า TOF ที่ด้านซ้าย
    if TOF_distance <= threshold_TOF:
        return True  # มีกำแพงด้านซ้าย
    else:
        return False  # ไม่มีผนังด้านซ้าย

# ฟังก์ชันตรวจสอบด้านขวา
def scan_gimbal_right():
    print('right')
    ep_gimbal.moveto(pitch=0, yaw=90, yaw_speed=300).wait_for_completed()
    time.sleep(1)
    # ตรวจสอบค่า TOF ที่ด้านขวา
    if TOF_distance <= threshold_TOF:
        return True  # มีกำแพงด้านขวา
    else:
        return False  # ไม่มีผนังด้านขวา

''' ----- Logic ----- '''
def logic():
    global status_logic

    left_status = scan_gimbal_left()
    front_status = scan_gimbal_front()
    right_status = scan_gimbal_right()
    ep_gimbal.recenter(yaw_speed=400).wait_for_completed()
    adjust_angle(yaw)
    time.sleep(0.5)

    # scan_gimbal_all()

    if front_status == False and right_status == True:
        # adjust_all_walls()
        move_forward()
        # adjust_all_walls()
        status_logic = 'move_forward'


    if right_status == False:
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        time.sleep(0.2)
        # adjust_front_wall()            
        turn_right()
        # adjust_front_wall()
        move_forward()
        status_logic = 'turn right'
                    

    if front_status == True and right_status == True and left_status == True:
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        move_stop()
        time.sleep(0.2)
        # adjust_all_walls()
        time.sleep(0.2)
        # adjust_all_walls()
        turn_back()
        # adjust_all_walls()s
        move_forward()

        status_logic = 'turn back'


    elif front_status == True and right_status == True:
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        time.sleep(0.2)
        # adjust_all_walls()
        turn_left()
        # adjust_all_walls()
        move_forward()

        status_logic = 'turn left'


''' ----- Display map ----- '''
def print_pretty_grid(grid):
    top_border = "__________________________________"
    bottom_border = "|_________________________________|"
    
    # print(top_border)
    
    for row in range(4):
        # Print top walls
        top_line = "|"
        mid_line = "|"
        bot_line = "|"
        for col in range(4):
            cell = grid[row][col]
            north_wall = cell[0]
            west_wall = cell[1]
            east_wall = cell[2]
            south_wall = cell[3]
            
            # Top wall (N)
            top_line += f" {'_' if north_wall == 2 else ' ' }   "
            
            # Middle line with walls (W to E)
            middle = ' V ' if cell[4] == 1 else ' ? '  # Use V for visited cells, ? for unvisited
            mid_line += f"{'|' if west_wall == 2 else ' '}{middle}{'|' if east_wall == 2 else ' '}"
            
            # Bottom wall (S)
            bot_line += f" {'_' if south_wall == 2 else ' ' }   "
        
        print(top_line + "|")
        print(mid_line + "|")
        print(bot_line + "|")
        print("|\t\t\t\t\t|")  # spacer line
    
    # print(bottom_border)

def check_all_cells_visited(grid):
    for row in range(4):
        for col in range(4):
            if grid[row][col][4] == 0:  # If any cell is not visited
                return False
    return True


if __name__ == "__main__":
    ep_robot = robot.Robot()
    print("Initializing robot...")
    ep_robot.initialize(conn_type="ap")

    ep_sensor = ep_robot.sensor
    ep_chassis = ep_robot.chassis
    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster
    ep_camera = ep_robot.camera
    ep_sensor_adaptor = ep_robot.sensor_adaptor
    time.sleep(2)

    center_x = 1280 / 2
    center_y = 720 / 2

    p = 0.6 / 1.7
    i = p / (0.7 / 2)
    d = p * (0.7 / 8)
    
    accumulate_err_x = 0
    accumulate_err_y = 0
    data_pitch_yaw = []
    prev_time = time.time()

    check = True
    count = 0


    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_sensor.sub_distance(freq=10, callback=tof_data_handler)
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)


    # ปรับตำแหน่ง Gimbal ให้ตรงศูนย์
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
    
    # adjust_all_walls()

    grid[cerrent_position[0]][cerrent_position[1]][4] = 1

    try:  
        while True:
            if TOF_distance is None or adc_left is None or adc_right is None:
                print("Waiting for sensor data...")
                time.sleep(1)
                continue

            maze_complete = False
            while not maze_complete:
                # update_wall()
                logic()
                # print_pretty_grid(grid)
                # print(grid)
                
                # Check if all cells have been visited
                if check_all_cells_visited(grid):
                    print("Maze exploration complete! All cells have been visited.")
                    maze_complete = True
                    # Stop the robot
                    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
                    break

                # adjust_angle(yaw)
                # time.sleep(0.5)
            

    except KeyboardInterrupt:
        print("Program stopped by user")
    # except Exception as e:
    #     print(f"An error occurred: {e}")
    finally:
        print("Cleaning up...")
        ep_chassis.unsub_position()
        ep_chassis.unsub_attitude()
        ep_sensor.unsub_distance()
        ep_chassis.drive_speed(x=0, y=0, z=0)
        ep_robot.close()
        print("Program ended...")