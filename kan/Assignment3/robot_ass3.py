import robomaster
from robomaster import robot
import math
import matplotlib.pyplot as plt
import time

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
row = 3
col = 3
cerrent_position = [row, col]
TOF_front_status = None
TOF_distance = 0

''' ----- TOF Sensor ----- '''
def tof_data_handler(sub_info):
    global TOF_distance
    TOF_distance = sub_info[0]
    # print(f"ToF distance: {tof_distance} mm")

    if TOF_distance <= 330:
        TOF_front_status = True

    else:
        TOF_front_status = False

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

    if 29 > adc_r_new:
        status_sharp_right = True
    else:
        status_sharp_right = False

    if 22 > adc_l_new:
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
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    time.sleep(0.5)

def move_forward():
    adjust_all_walls()
    ep_chassis.move(x=0.6, y=0, z=0, xy_speed=MAX_SPEED).wait_for_completed()
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
    print("Movement: Drive forward")

def turn_back():
    ep_chassis.move(x=0, y=0, z=180, xy_speed=MAX_SPEED).wait_for_completed()
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
    print("Movement: Turn Back")

def turn_left():
    ep_chassis.move(x=0, y=0, z=90, xy_speed=MAX_SPEED).wait_for_completed()
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
    print("Movement: Turn Left") 
    

def turn_right():
    ep_chassis.move(x=0, y=0, z=-90, xy_speed=MAX_SPEED).wait_for_completed()
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


''' ----- Update_Maze ----- '''
status_logic = None
def update_wall():
    global cerrent_position,status_logic
    
    getDirectionFacing()
    logic()

    if robot_status_now == 'N':
        if status_logic == 'move_forward':
            grid[cerrent_position[0]][cerrent_position[1]][2] = 2 # มีกำแพงด้านขวาที่พิกัด x, y
            grid[cerrent_position[0]-1][cerrent_position[1]][4] = 1 # 1 คือเคยไปช่องนั้นมาแล้ว

            cerrent_position = [cerrent_position[0]-1]

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
            grid[cerrent_position[0]-1][cerrent_position[1]][4] = 1
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


''' ----- Logic ----- '''
def logic():
    global status_logic
    if TOF_front_status == False and status_sharp_right == True:
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        time.sleep(0.2)
        adjust_front_wall()            
        move_forward()
        adjust_front_wall()
        status_logic = 'move_forward'
                
    elif status_sharp_right == False:            
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        time.sleep(0.2)
        adjust_all_walls()
        turn_right()
        adjust_left_wall()
        adjust_front_wall()
        move_forward()
        status_logic = 'turn right'

    elif TOF_front_status == True and status_sharp_right == True and status_sharp_left == True:
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        time.sleep(0.2)
        adjust_all_walls()
        ep_gimbal.moveto(pitch=0, yaw=-90, pitch_speed=100, yaw_speed=100).wait_for_completed()
        tof_check_left = TOF_distance
        if tof_check_left < 400:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            time.sleep(0.2)
            adjust_all_walls()
            turn_back()
            adjust_right_wall()
            adjust_front_wall()
            move_forward()
            status_logic = 'turn back'
                    
        else:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            time.sleep(0.2)
            adjust_all_walls()
            turn_left()
            adjust_right_wall()
            adjust_front_wall()
            move_forward()
            status_logic = 'turn left'
                        

    elif TOF_front_status == True and status_sharp_right == True:
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        time.sleep(0.2)
        adjust_all_walls()
        ep_gimbal.moveto(pitch=0, yaw=-90, pitch_speed=100, yaw_speed=100).wait_for_completed()
        tof_check_left = TOF_distance
        if tof_check_left < 400:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            time.sleep(0.2)
            adjust_all_walls()
            turn_back()
            adjust_right_wall()
            adjust_front_wall()
            move_forward()
            status_logic = 'turn back'
                    
        else:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            time.sleep(0.2)
            adjust_all_walls()
            turn_left()
            adjust_right_wall()
            adjust_front_wall()
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
    animate_loading("Initializing robot", duration=5)
    ep_robot.initialize(conn_type="ap")

    ep_sensor = ep_robot.sensor
    ep_chassis = ep_robot.chassis
    ep_gimbal = ep_robot.gimbal
    ep_sensor_adaptor = ep_robot.sensor_adaptor
    time.sleep(2)

    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_sensor.sub_distance(freq=10, callback=tof_data_handler)
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)

    # ปรับตำแหน่ง Gimbal ให้ตรงศูนย์
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
    
    adjust_all_walls()

    grid[cerrent_position[0]][cerrent_position[1]][4] = 1

    try:  
        while True:
            if TOF_distance is None or adc_left is None or adc_right is None:
                print("Waiting for sensor data...")
                time.sleep(1)
                continue

            maze_complete = False
            while not maze_complete:
                update_wall()
                print_pretty_grid(grid)
                # print(grid)
                
                # Check if all cells have been visited
                if check_all_cells_visited(grid):
                    print("Maze exploration complete! All cells have been visited.")
                    maze_complete = True
                    # Stop the robot
                    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
                    break
                

    except KeyboardInterrupt:
        print("Program stopped by user")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        animate_loading("Cleaning up", duration=5)
        ep_chassis.unsub_position()
        ep_chassis.unsub_attitude()
        ep_sensor.unsub_distance()
        ep_chassis.drive_speed(x=0, y=0, z=0)
        ep_robot.close()
        animate_loading("Program ended", duration=5)