import robomaster
from robomaster import robot, blaster, camera
import math
import matplotlib.pyplot as plt
import time
import cv2
import numpy as np
import pandas as pd

# Function to load list_travel from CSV
def load_travel_data(filename="list_travel.csv"):
    try:
        # Read the CSV file
        df_travel = pd.read_csv(filename)
        
        # Extract the 'travel' column and convert it to a list of tuples
        list_travel = [eval(item) for item in df_travel['travel']]
        
        print(f"Travel data loaded from {filename}")
        return list_travel
    except FileNotFoundError:
        print(f"File {filename} not found!")
        return []
    
MAX_SPEED = 10

# start map
row = 5
col = 2
cerrent_po = [row,col]
start_position = [5, 2]
list_travel = load_travel_data()
# list_chick = []
# list_acrylic = []


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

''' ----- movement ----- '''
def move_stop():
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.75)
    time.sleep(0.7)

def move_forward():
    print("Drive forward")
    ep_chassis.move(x=0.57, y=0, z=0, xy_speed=MAX_SPEED).wait_for_completed()
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

    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_sensor.sub_distance(freq=10, callback=tof_data_handler)
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)

    # ปรับตำแหน่ง Gimbal ให้ตรงศูนย์
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()

    ep_blaster.set_led(brightness=255, effect=blaster.LED_OFF)
    ep_chassis.unsub_position()
    ep_chassis.unsub_attitude()
    ep_sensor.unsub_distance()
    ep_chassis.drive_speed(x=0, y=0, z=0)
    ep_robot.close()

