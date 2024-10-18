import robomaster
from robomaster import robot, blaster, camera
import math
import matplotlib.pyplot as plt
import time
import cv2
import numpy as np
import pandas as pd

MAX_SPEED = 10
#0 = front wall from north
#1 = left wall from north
#2 = right wall from north
#3 = back wall from north
#4 = visit
#[North(front), West(left), East(right), South(back),Visited,chicken,thive]
grid = [
    [[2,2,0,0,0,0,0],[2,0,0,0,0,0,0],[2,0,0,0,0,0,0],[2,0,0,0,0,0,0],[2,0,0,0,0,0,0],[2,0,2,0,0,0,0]],
    [[0,2,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,2,0,0,0,0]],
    [[0,2,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,2,0,0,0,0]],
    [[0,2,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,2,0,0,0,0]],
    [[0,2,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,2,0,0,0,0]],
    [[0,2,0,2,0,0,0],[0,0,0,2,0,0,0],[0,0,0,2,0,0,0],[0,0,0,2,0,0,0],[0,0,0,2,0,0,0],[0,0,2,2,0,0,0]]
]

# start map
row = 5
col = 2
cerrent_po = [row,col]
list_travel = []
direction_robot = []
list_chick = []
list_acrylic = []


# Function to save list_travel to CSV, including the start column
def save_travel_data(direction_robot, list_travel, filename="list_path.csv"):
    # Create DataFrame without repeating start_position
    df_travel = pd.DataFrame({"direction": direction_robot, "travel": list_travel})
    # Save the travel data to CSV
    df_travel.to_csv(filename, index=False)
    # Add the start_position to the first row manually

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
        dp=1.2, 
        minDist=50, 
        param1=50, 
        param2=30, 
        minRadius=10, 
        maxRadius=40
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

                    x,y,w,h = top_left[0]-15, top_left[1]-5, w+27, h+60 
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
    img2 = cv2.imread('current_detection.png')

    orb = cv2.ORB_create()

    keypoints1, descriptors1 = orb.detectAndCompute(img1, None)
    keypoints2, descriptors2 = orb.detectAndCompute(img2, None)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(descriptors1, descriptors2)
    matches = sorted(matches, key=lambda x: x.distance)
    similarity_score = len([m for m in matches if m.distance < 50])  
    print(f"คะแนนความคล้ายของรูปภาพคือ: {similarity_score}")

    if similarity_score >= 0: 
        # ep_blaster.fire(fire_type=blaster.INFRARED_FIRE, times=2)
        ep_blaster.fire(times=5)

# def detect_chicken(frame):
#     hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

#     lower_chicken = np.array([29, 235, 85])
#     upper_chicken = np.array([37, 255, 255])

#     mask_chicken = cv2.inRange(hsv_frame, lower_chicken, upper_chicken)
#     contours_chicken, _ = cv2.findContours(mask_chicken, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#     if contours_chicken: 
#         max_chicken_contour = max(contours_chicken, key=cv2.contourArea)
#         (x_chicken, y_chicken, w_chicken, h_chicken) = cv2.boundingRect(max_chicken_contour)
        
#         if w_chicken > 50:
#                 add_w_chicken = int(w_chicken * 0.3)
#                 add_h_chicken = int(h_chicken * 0.6)
#                 new_y_chicken = y_chicken - add_h_chicken // 2 + 20

#         else :
#                 add_w_chicken = int(w_chicken * 0.35)
#                 add_h_chicken = int(h_chicken * 0.55)
#                 new_y_chicken = (y_chicken - add_h_chicken // 2 + 20) - 15

#         new_x_chicken = x_chicken - add_w_chicken // 2
#         new_w_chicken = w_chicken + add_w_chicken
#         new_h_chicken = h_chicken + add_h_chicken

#         cv2.rectangle(frame, (new_x_chicken, new_y_chicken), (new_x_chicken + new_w_chicken, new_y_chicken + new_h_chicken), (255, 0, 255), 2)
#         cv2.putText(frame, f" chicken x: {x_chicken}, y: {y_chicken}, w: {w_chicken}, h: {h_chicken}", (x_chicken, y_chicken - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
#         print('>>>>>chick<<<<<<')

def detect():
    global status_logic, count, check, p, frame, marker, accumulate_err_x ,accumulate_err_y, data_pitch_yaw, prev_time 
    while True:
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        marker = detect_blue_circles(frame, count)
        # detect_chicken(frame)
        if marker and marker._w >= 30 and marker._w < 100 and count == 0: check = False
        if count > 0:count += 1
        print('--->',check)
        if check: break

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
            if count == 10:
                temp_image_path = 'temp_frame.png'
                cv2.imwrite(temp_image_path, frame)
                process_image(
                    image_path=temp_image_path,
                    output_path='current_detection.png',
                    alpha=1.5,
                    beta=0,
                    canny_threshold1=40,
                    canny_threshold2=160,
                    dilation_kernel_size=(3, 3),
                    dilation_iterations=1
                )
            elif count == 15:
                matches_template()
            elif count >=20:
                ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
                ep_blaster.set_led(brightness=0, effect=blaster.LED_OFF)
                check = True
                count = 0
                print('---------')
                break
            else:
                ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)



''' ----- TOF Sensor ----- '''
status_tof = None
tof_distance = 0
def tof_data_handler(sub_info):
    global tof_distance, status_tof
    tof_distance = sub_info[0]
    # print(f"ToF distance: {tof_distance} mm")
    if 300 >= tof_distance >= 100:
        status_tof = True
    else:
        status_tof = False
    sharp_sen_data()


''' ----- Sharp Sensor ----- '''
status_ss_l = None
adc_l = None
adc_l_new = 0

status_ss_r = None
adc_r = None
adc_r_new = 0
def sharp_sen_data():
    global adc_r,adc_l,adc_r_new,adc_l_new,status_ss_l,status_ss_r
    adc_r = ep_sensor_adaptor.get_adc(id=2, port=2)
    adc_r_cm = (adc_r * 3) / 1023  # process to cm unit
    adc_l = ep_sensor_adaptor.get_adc(id=1, port=1)
    adc_l_cm = (adc_l * 3) / 1023  # process to cm unit
    # print(f"adc_r_cm: {adc_r_cm} volt")
    # print(f"adc_l_cm: {adc_l_cm} volt")
    if adc_r_cm > 1.4:
        adc_r_new = ((adc_r_cm - 4.2) / -0.31)
    elif 1.4 >= adc_r_cm >= 0.6:
        adc_r_new = ((adc_r_cm - 2.03) / -0.07)
    elif 0 <= adc_r_cm < 0.6:
        adc_r_new = ((adc_r_cm - 0.95) / -0.016)

    if adc_l_cm > 1.4:
        adc_l_new = ((adc_l_cm - 4.2) / -0.31)
    elif 1.4 >= adc_l_cm >= 0.6:
        adc_l_new = ((adc_l_cm - 2.03) / -0.07)
    elif 0 <= adc_l_cm < 0.6:
        adc_l_new = ((adc_l_cm - 0.95) / -0.016)
    
    if 46 > adc_r_new > 2:
        status_ss_r = True
    else:
        status_ss_r = False

    if 40 > adc_l_new > 2:
        status_ss_l = True
    else:
        status_ss_l = False

''' ----- sub_position_handler ----- '''
x = 0
y = 0
# ฟังก์ชันสำหรับจัดการตำแหน่งของหุ่นยนต์
def sub_position_handler(position_info):
    global x,y
    x, y, z = position_info


''' ----- sub_attitude_info_handler ----- '''
yaw = 0
threshold_sharp = 30

# ฟังก์ชันสำหรับจัดการท่าทางของหุ่นยนต์
def sub_attitude_info_handler(attitude_info):
    global yaw
    yaw, pitch, roll = attitude_info
    # print(f"chassis attitude: yaw:{yaw}")


''' ----- adjust_all_walls ----- '''
def adjust_wall():
    adjust_wall_f()
    adjust_wall_l()
    adjust_wall_r()

def adjust_wall_l():

    if adc_l_new < 40:
        if adc_l_new < 25:
            walk_y = (abs(25 - adc_l_new)/100)-0.035
            # print("Move right")
            ep_chassis.move(x=0, y=walk_y, z=0, xy_speed=MAX_SPEED).wait_for_completed()
        elif adc_l_new > 35:
            walk_y = (abs(35 - adc_l_new)/100)-0.035
            # print("Move left")
            ep_chassis.move(x=0, y=-walk_y, z=0, xy_speed=MAX_SPEED).wait_for_completed()

def adjust_wall_r():

    if adc_r_new < 40:
        if adc_r_new < 25 :
            walk_y = (abs(25 - adc_r_new)/100)-0.035
            # print("Move left")
            ep_chassis.move(x=0, y=-walk_y, z=0, xy_speed=MAX_SPEED).wait_for_completed()
        
        elif adc_r_new > 35:
            walk_y = (abs(35 - adc_r_new)/100)-0.035
            # print("Move right")
            ep_chassis.move(x=0, y=walk_y, z=0, xy_speed=MAX_SPEED).wait_for_completed()

def adjust_wall_f():    
    
    if tof_distance <150:
        walk_x = (abs(150-tof_distance)/1000)+0.02
        ep_chassis.move(x=-walk_x, y=0, z=0, xy_speed=MAX_SPEED).wait_for_completed()
    
    if 450>=tof_distance>300:
        walk_x = (abs(tof_distance -300)/1000)+0.02
        ep_chassis.move(x=walk_x, y=0, z=0, xy_speed=MAX_SPEED).wait_for_completed()


''' ----- movement ----- '''
def move_stop():
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.75)
    time.sleep(0.7)

def move_forward():
    print("Drive forward")
    # default speed อยู่ที่ 0.57 เปลี่ยนเป็น 0.56
    ep_chassis.move(x=0.56, y=0, z=0, xy_speed=MAX_SPEED).wait_for_completed()
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
    

def turn_back():
    print("Turn Back")
    
    ep_chassis.move(x=0, y=0, z=180, xy_speed=MAX_SPEED).wait_for_completed()
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
    

def turn_left():
    print("Turn Left")
    
    ep_chassis.move(x=0, y=0, z=90, xy_speed=MAX_SPEED).wait_for_completed()
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
    

def turn_right():
    print('Turn Right')
    
    ep_chassis.move(x=0, y=0, z=-90, xy_speed=MAX_SPEED).wait_for_completed()
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
    
    
''' ----- DirectionFacing ----- '''
robo_status_now = None
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


''' ----- Update_Maze ----- '''
status_logic = None
def update_wall():
    global cerrent_po,status_logic
    list_travel.append(tuple(cerrent_po))
    getDirectionFacing()
    direction_robot.append(robo_status_now)
    logic()
    if robo_status_now =='N':
        if status_logic == 'move_forward':
            grid[cerrent_po[0]-1][cerrent_po[1]][4] = 1

            cerrent_po = [cerrent_po[0]-1,cerrent_po[1]]
    
        if status_logic == 'turn right': #turn right and move forward
            grid[cerrent_po[0]][cerrent_po[1]+1][4] = 1
            
            cerrent_po = [cerrent_po[0],cerrent_po[1]+1]
        
        if status_logic == 'turn back':
            grid[cerrent_po[0]+1][cerrent_po[1]][4] = 1

            cerrent_po = [cerrent_po[0]+1,cerrent_po[1]]

        if status_logic == 'turn left':
            grid[cerrent_po[0]][cerrent_po[1]-1][4] = 1

            cerrent_po = [cerrent_po[0],cerrent_po[1]-1]       

    if robo_status_now =='E':  
        if status_logic == 'move_forward':
            grid[cerrent_po[0]][cerrent_po[1]+1][4] = 1

            cerrent_po = [cerrent_po[0],cerrent_po[1]+1]
    
        if status_logic == 'turn right': #turn right and move forward
            grid[cerrent_po[0]+1][cerrent_po[1]][4] = 1
            
            cerrent_po = [cerrent_po[0]+1,cerrent_po[1]]
        
        if status_logic == 'turn back':
            grid[cerrent_po[0]][cerrent_po[1]-1][4] = 1

            cerrent_po = [cerrent_po[0],cerrent_po[1]-1]

        if status_logic == 'turn left':
            grid[cerrent_po[0]-1][cerrent_po[1]][4] = 1

            cerrent_po = [cerrent_po[0]-1,cerrent_po[1]]

    if robo_status_now =='S':
        if status_logic == 'move_forward':
            grid[cerrent_po[0]+1][cerrent_po[1]][4] = 1


            cerrent_po = [cerrent_po[0]+1,cerrent_po[1]]
    
        if status_logic == 'turn right': #turn right and move forward
            grid[cerrent_po[0]][cerrent_po[1]-1][4] = 1
            
            cerrent_po = [cerrent_po[0],cerrent_po[1]-1]
        
        if status_logic == 'turn back':
            grid[cerrent_po[0]+1][cerrent_po[1]][4] = 1

            cerrent_po = [cerrent_po[0]+1,cerrent_po[1]]

        if status_logic == 'turn left':
            grid[cerrent_po[0]][cerrent_po[1]+1][4] = 1
            cerrent_po = [cerrent_po[0],cerrent_po[1]+1]      

    if robo_status_now =='W':
        if status_logic == 'move_forward':
            grid[cerrent_po[0]][cerrent_po[1]-1][4] = 1

            cerrent_po = [cerrent_po[0],cerrent_po[1]-1]
    
        if status_logic == 'turn right': #turn right and move forward
            grid[cerrent_po[0]-1][cerrent_po[1]][4] = 1
            
            cerrent_po = [cerrent_po[0]-1,cerrent_po[1]]
        
        if status_logic == 'turn back':
            grid[cerrent_po[0]][cerrent_po[1]+1][4] = 1

            cerrent_po = [cerrent_po[0],cerrent_po[1]+1]

        if status_logic == 'turn left':
            grid[cerrent_po[0]+1][cerrent_po[1]][4] = 1

            cerrent_po = [cerrent_po[0]+1,cerrent_po[1]]


status_logic = None
def check_tof_wall(tof_now):
    global cerrent_po,status_logic
    if tof_now ==True:

        if robo_status_now =='N':
            grid[cerrent_po[0]][cerrent_po[1]][0] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have west wall             

        if robo_status_now =='E':  
            grid[cerrent_po[0]][cerrent_po[1]][2] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have north wall             

        if robo_status_now =='S':
            grid[cerrent_po[0]][cerrent_po[1]][3] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have east wall   

        if robo_status_now =='W':
            grid[cerrent_po[0]][cerrent_po[1]][1] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have south wall 

def check_left_wall(sharp_l_now):
    global cerrent_po,status_logic
    if sharp_l_now ==True:

        if robo_status_now =='N':
            grid[cerrent_po[0]][cerrent_po[1]][1] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have west wall             

        if robo_status_now =='E':  
            grid[cerrent_po[0]][cerrent_po[1]][0] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have north wall             

        if robo_status_now =='S':
            grid[cerrent_po[0]][cerrent_po[1]][2] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have east wall   

        if robo_status_now =='W':
            grid[cerrent_po[0]][cerrent_po[1]][3] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have south wall  

def check_right_wall(sharp_r_now):
    global cerrent_po,status_logic
    if sharp_r_now == True:
        if robo_status_now =='N':
            grid[cerrent_po[0]][cerrent_po[1]][2] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have east wall             

        if robo_status_now =='E':  
            grid[cerrent_po[0]][cerrent_po[1]][3] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have back wall             

        if robo_status_now =='S':
            grid[cerrent_po[0]][cerrent_po[1]][1] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have west wall   

        if robo_status_now =='W':
            grid[cerrent_po[0]][cerrent_po[1]][0] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have front wall  
    
def check_all_wall (tof,l,r):
    check_tof_wall(tof)
    check_left_wall(l)
    check_right_wall(r)

f_wall = None
l_wall = None
r_wall = None
def check_2_wall():
    global f_wall,l_wall,r_wall
    if robo_status_now =='N':
        if grid[cerrent_po[0]][cerrent_po[1]][0] == 2:
            f_wall = True
        else:
            f_wall = False
        
        if grid[cerrent_po[0]][cerrent_po[1]][1] == 2:
            l_wall = True
        else:
            l_wall = False

        if grid[cerrent_po[0]][cerrent_po[1]][2] == 2:
            r_wall = True
        else:
            r_wall = False

        if grid[cerrent_po[0]][cerrent_po[1]][2] == 2:
            b_wall = True
        else:
            b_wall = False
        
    
    if robo_status_now =='E':
        if grid[cerrent_po[0]][cerrent_po[1]][0] == 2:
            l_wall = True
        
        else:
            l_wall = False

        if grid[cerrent_po[0]][cerrent_po[1]][2] == 2:
            f_wall = True
        
        else:
            f_wall = False
        
        if grid[cerrent_po[0]][cerrent_po[1]][3] == 2:
            r_wall = True
        
        else:
            r_wall = False


    if robo_status_now =='S':
        
        if grid[cerrent_po[0]][cerrent_po[1]][1] == 2:
            r_wall = True
        
        else:
            r_wall = False

        if grid[cerrent_po[0]][cerrent_po[1]][2] == 2:
            l_wall = True   
        
        else:
            l_wall = False

        if grid[cerrent_po[0]][cerrent_po[1]][3] == 2:
            f_wall = True
        
        else:
            f_wall = False

    if robo_status_now =='W':
        if grid[cerrent_po[0]][cerrent_po[1]][0] == 2:
            r_wall = True
        
        else:
            r_wall = False
        
        if grid[cerrent_po[0]][cerrent_po[1]][1] == 2:
            f_wall = True
        
        else:
            f_wall = False

        if grid[cerrent_po[0]][cerrent_po[1]][3] == 2:
            l_wall = True
        
        else:
            l_wall = False

''' ----- Logic ----- '''
def logic():
    global status_logic, check 
    ep_gimbal.moveto(pitch=0, yaw=-90, pitch_speed=100, yaw_speed=100).wait_for_completed()
    sl = tof_distance
    ep_gimbal.moveto(pitch=0, yaw= 0, pitch_speed=100, yaw_speed=100).wait_for_completed()
    tof = tof_distance
    ep_gimbal.moveto(pitch=0, yaw= 90, pitch_speed=100, yaw_speed=100).wait_for_completed()
    sr = tof_distance
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
    print(f'L{sl} , F{tof} , R{sr}')
    if sl <400:
        status_sl = True
    else:
        status_sl = False
    
    if sr <400:
        status_sr = True
    
    else:
        status_sr = False
    
    if tof <400:
        status_of_tof = True
    
    else:
        status_of_tof = False
    
    check_all_wall(status_of_tof,status_sl,status_sr)
    check_2_wall()
    print(robo_status_now)
    print(f'L_wall:{l_wall} , F_wall:{f_wall} , R_wall:{r_wall}')
    if f_wall == False and r_wall == True:
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        adjust_wall_f()
        move_forward()
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        status_logic = 'move_forward'
    elif r_wall == False:     
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)       
        adjust_wall_f()
        turn_right()
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        move_forward()
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        status_logic = 'turn right'

    elif f_wall == True and r_wall == True and l_wall == True:
        adjust_wall_f()
        turn_back()
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        move_forward()
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        status_logic = 'turn back'
    elif f_wall == True and r_wall == True:
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        adjust_wall_f()
        turn_left()
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0) 
        move_forward()
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        status_logic = 'turn left'
    
    print('current_position =', cerrent_po)
    print('direction_robot =', robo_status_now)

''' ----- Display map ----- '''
def print_pretty_grid(grid):
    
    # print(top_border)
    
    for row in range(6):
        # Print top walls
        top_line = "|"
        mid_line = "|"
        bot_line = "|"
        for col in range(6):
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
    for row in range(6):
        for col in range(6):
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

    p = 0.35
    i = p / (0.7 / 2)
    d = p * (0.7 / 8)
    # p = 0.4705
    # i = 1.1192
    # d = 0.0494

    accumulate_err_x = 0
    accumulate_err_y = 0
    data_pitch_yaw = []
    prev_time = time.time()

    check = True
    count = 0

    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_sensor.sub_distance(freq=10, callback=tof_data_handler)
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)
    # ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)


    # ปรับตำแหน่ง Gimbal ให้ตรงศูนย์
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
    adjust_wall()
    # adjust_all_walls()

    grid[cerrent_po[0]][cerrent_po[1]][4] = 1
    try:  

        maze_complete = False
        while not maze_complete:
            update_wall()
            # print('chick:',found_chick)
            # if found_chick:
            #     list_chick.append(tuple(cerrent_position))
            #     found_chick = False
            # if found_acrylic:
            #     list_acrylic.append(tuple(cerrent_position))
            #     found_acrylic = False
            if check_all_cells_visited(grid) :
                print("Maze exploration complete! All cells have been visited.")
                maze_complete = True
                ep_chassis.drive_speed(x=0, y=0, z=0)
                ep_robot.close()

                # Save the travel data to CSV
                save_travel_data(direction_robot, list_travel)
                break
            

    except KeyboardInterrupt:
        print("Program stopped by user. Saving travel data before exit...")
        save_travel_data(direction_robot, list_travel)  # Save data when interrupted
    except Exception as e:
        print(f"An error occurred: {e}")
        save_travel_data(direction_robot, list_travel)  # Save data in case of error
    finally:
        print("Cleaning up...")
        print(grid)
        print_pretty_grid(grid)
        ep_blaster.set_led(brightness=255, effect=blaster.LED_OFF)
        ep_chassis.unsub_position()
        ep_chassis.unsub_attitude()
        ep_sensor.unsub_distance()
        ep_chassis.drive_speed(x=0, y=0, z=0)
        ep_robot.close()
        print("Program ended...")