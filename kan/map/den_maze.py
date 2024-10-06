import robomaster
from robomaster import robot
import time


MAX_SPEED = 15
#0 = front wall from north
#1 = left wall from north
#2 = right wall from north
#3 = back wall from north
#4 = visit
#[North(front), West(left), East(right), South(back),Visited]
grid = [
        [[2,2,0,0,0],[2,0,0,0,0],[2,0,0,0,0],[2,0,2,0,0]],
        [[0,2,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,2,0,0]],
        [[0,2,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,2,0,0]],
        [[0,2,0,2,0],[0,0,0,2,0],[0,0,0,2,0],[0,0,2,2,0]]
        ]

row = 3
col = 1
cerrent_po = [row,col]
status_tof = None
tof_distance = 0
def tof_data_handler(sub_info):
    global tof_distance, status_tof
    tof_distance = sub_info[0]
    print(f"ToF distance: {tof_distance} mm")
    if 300 >= tof_distance >= 100:
        status_tof = True
    else:
        status_tof = False
    
    sharp_sen_data()


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
    adc_l = ep_sensor_adaptor.get_adc(id=3, port=1)
    adc_l_cm = (adc_l * 3) / 1023  # process to cm unit
    # print(f"adc_r_cm: {adc_r_cm} volt")
    # print(f"adc_l_cm: {adc_l_cm} volt")
    if adc_r_cm > 1.4:
        adc_r_new = ((adc_r_cm - 4.2) / -0.31)
    elif 1.4 >= adc_r_cm >= 0.6:
        adc_r_new = ((adc_r_cm - 2.03) / -0.07)
    elif 0 <= adc_r_cm < 0.6:
        adc_r_new = ((adc_r_cm - 0.95) / -0.016)-7

    if adc_l_cm > 1.4:
        adc_l_new = ((adc_l_cm - 4.2) / -0.31)
    elif 1.4 >= adc_l_cm >= 0.6:
        adc_l_new = ((adc_l_cm - 2.03) / -0.07)
    elif 0 <= adc_l_cm < 0.6:
        adc_l_new = ((adc_l_cm - 0.95) / -0.016)-7
    
    if 35 > adc_r_new > 2:
        status_ss_r = True
    else:
        status_ss_r = False

    if 30 > adc_l_new > 2:
        status_ss_l = True
    else:
        status_ss_l = False

    # print(f"distance from front wall:right  {adc_r} left  {adc_l}")
    print(f"distance from front wall:left  {adc_l_new} right  {adc_r_new}")

x = 0
y = 0
# ฟังก์ชันสำหรับจัดการตำแหน่งของหุ่นยนต์
def sub_position_handler(position_info):
    global x,y
    x, y, z = position_info


yaw = 0
# ฟังก์ชันสำหรับจัดการท่าทางของหุ่นยนต์
def sub_attitude_info_handler(attitude_info):
    global yaw
    yaw, pitch, roll = attitude_info
    # print(f"chassis attitude: yaw:{yaw}")

# def move_first():
#     ep_chassis.move(x=0.15, y=0, z=0, xy_speed=MAX_SPEED).wait_for_completed()
#     ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
#     adjust_wall_r()
#     adjust_wall_f()

def move_forward():
    print("Drive forward")
    
    adjust_wall()
    ep_chassis.move(x=0.6, y=0, z=0, xy_speed=MAX_SPEED).wait_for_completed()
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
    adjust_wall_r()
    adjust_wall_f()
    

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
    


def adjust_wall():
    adjust_wall_f()
    adjust_wall_l()
    adjust_wall_r()


def adjust_wall_l():
    walk_y = abs(15 - adc_l_new)/100
    if adc_l_new < 30:
        if adc_l_new < 15:
            print("Move right")
            ep_chassis.move(x=0, y=walk_y, z=0, xy_speed=MAX_SPEED).wait_for_completed()
        elif adc_l_new > 15:
            print("Move left")
            ep_chassis.move(x=0, y=-walk_y, z=0, xy_speed=MAX_SPEED).wait_for_completed()

def adjust_wall_r():
    walk_y = (abs(17 - adc_r_new)/100)-0.01
    if adc_r_new < 34:
        if adc_r_new < 17 :
            print("Move left")
            ep_chassis.move(x=0, y=-walk_y, z=0, xy_speed=MAX_SPEED).wait_for_completed()
        
        elif adc_r_new > 17:
            print("Move right")
            ep_chassis.move(x=0, y=walk_y, z=0, xy_speed=MAX_SPEED).wait_for_completed()

def adjust_wall_f():    
    
    if tof_distance <100:
        walk_x = abs(100-tof_distance)/1000
        ep_chassis.move(x=-walk_x, y=0, z=0, xy_speed=MAX_SPEED).wait_for_completed()
    
    if 400>=tof_distance>300:
        walk_x = (tof_distance -300)/1000
        ep_chassis.move(x=walk_x, y=0, z=0, xy_speed=MAX_SPEED).wait_for_completed()

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

status_logic = None
def update_wall():
    global cerrent_po,status_logic
    logic()
    if robo_status_now =='N':
        if status_logic == 'move_forward':
            grid[cerrent_po[0]][cerrent_po[1]][2] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have right wall 
            grid[cerrent_po[0]+1][cerrent_po[1]][4] = 1
            # grid[cerrent_po[0]+1][cerrent_po[1]][3] = 1


            cerrent_po = [cerrent_po[0]-1,cerrent_po[1]]
    
        if status_logic == 'turn right': #turn right and move forward
            # grid[cerrent_po[0]][cerrent_po[1]][2] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have right wall 
            grid[cerrent_po[0]][cerrent_po[1]+1][4] = 1
            
            cerrent_po = [cerrent_po[0],cerrent_po[1]+1]
        
        if status_logic == 'turn back':
            grid[cerrent_po[0]][cerrent_po[1]][0] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have front wall
            grid[cerrent_po[0]][cerrent_po[1]][1] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have left wall
            grid[cerrent_po[0]][cerrent_po[1]][2] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have right wall 

            grid[cerrent_po[0]+1][cerrent_po[1]][4] = 1
            # grid[cerrent_po[0]+1][cerrent_po[1]][3] = 1

            cerrent_po = [cerrent_po[0]+1,cerrent_po[1]]

        if status_logic == 'turn left':
            grid[cerrent_po[0]][cerrent_po[1]][0] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have front wall
            grid[cerrent_po[0]][cerrent_po[1]][2] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have right wall 
            grid[cerrent_po[0]-1][cerrent_po[1]][4] = 1
            # grid[cerrent_po[0]+1][cerrent_po[1]][3] = 1 
            cerrent_po = [cerrent_po[0],cerrent_po[1]-1]       

    if robo_status_now =='E':
        if status_logic == 'move_forward':
            grid[cerrent_po[0]][cerrent_po[1]][0] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have front wall 
            grid[cerrent_po[0]][cerrent_po[1]+1][4] = 1
            # grid[cerrent_po[0]+1][cerrent_po[1]][3] = 1


            cerrent_po = [cerrent_po[0],cerrent_po[1]+1]
    
        if status_logic == 'turn right': #turn right and move forward
            # grid[cerrent_po[0]][cerrent_po[1]][2] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have right wall 
            grid[cerrent_po[0]+1][cerrent_po[1]][4] = 1
            
            cerrent_po = [cerrent_po[0]+1,cerrent_po[1]]
        
        if status_logic == 'turn back':
            grid[cerrent_po[0]][cerrent_po[1]][0] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have front wall
            grid[cerrent_po[0]][cerrent_po[1]][2] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have right wall
            grid[cerrent_po[0]][cerrent_po[1]][3] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have back wall

            grid[cerrent_po[0]][cerrent_po[1]-1][4] = 1
            # grid[cerrent_po[0]+1][cerrent_po[1]][3] = 1

            cerrent_po = [cerrent_po[0],cerrent_po[1]-1]

        if status_logic == 'turn left':
            grid[cerrent_po[0]][cerrent_po[1]][2] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have right wall 
            grid[cerrent_po[0]][cerrent_po[1]][3] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have back wall 
            grid[cerrent_po[0]-1][cerrent_po[1]][4] = 1
            # grid[cerrent_po[0]+1][cerrent_po[1]][3] = 1 
            cerrent_po = [cerrent_po[0]-1,cerrent_po[1]]

    if robo_status_now =='S':
        if status_logic == 'move_forward':
            grid[cerrent_po[0]][cerrent_po[1]][1] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have left wall 
            grid[cerrent_po[0]+1][cerrent_po[1]][4] = 1
            # grid[cerrent_po[0]+1][cerrent_po[1]][3] = 1


            cerrent_po = [cerrent_po[0]+1,cerrent_po[1]]
    
        if status_logic == 'turn right': #turn right and move forward
            # grid[cerrent_po[0]][cerrent_po[1]][2] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have right wall 
            grid[cerrent_po[0]][cerrent_po[1]-1][4] = 1
            
            cerrent_po = [cerrent_po[0],cerrent_po[1]-1]
        
        if status_logic == 'turn back':
            
            grid[cerrent_po[0]][cerrent_po[1]][1] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have left wall
            grid[cerrent_po[0]][cerrent_po[1]][2] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have right wall 
            grid[cerrent_po[0]][cerrent_po[1]][3] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have back wall

            grid[cerrent_po[0]+1][cerrent_po[1]][4] = 1
            # grid[cerrent_po[0]+1][cerrent_po[1]][3] = 1

            cerrent_po = [cerrent_po[0]+1,cerrent_po[1]]

        if status_logic == 'turn left':
            grid[cerrent_po[0]][cerrent_po[1]][1] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have left wall 
            grid[cerrent_po[0]][cerrent_po[1]][3] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have back wall 
            grid[cerrent_po[0]][cerrent_po[1]+1][4] = 1
            # grid[cerrent_po[0]+1][cerrent_po[1]][3] = 1 
            cerrent_po = [cerrent_po[0],cerrent_po[1]+1]      

    if robo_status_now =='W':
        if status_logic == 'move_forward':
            grid[cerrent_po[0]][cerrent_po[1]][0] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have back wall 
            grid[cerrent_po[0]][cerrent_po[1]-1][4] = 1
            # grid[cerrent_po[0]+1][cerrent_po[1]][3] = 1


            cerrent_po = [cerrent_po[0],cerrent_po[1]-1]
    
        if status_logic == 'turn right': #turn right and move forward
            # grid[cerrent_po[0]][cerrent_po[1]][2] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have right wall 
            grid[cerrent_po[0]-1][cerrent_po[1]][4] = 1
            
            cerrent_po = [cerrent_po[0]-1,cerrent_po[1]]
        
        if status_logic == 'turn back':
            grid[cerrent_po[0]][cerrent_po[1]][0] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have front wall
            grid[cerrent_po[0]][cerrent_po[1]][1] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have left wall
            grid[cerrent_po[0]][cerrent_po[1]][3] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have back wall

            grid[cerrent_po[0]][cerrent_po[1]+1][4] = 1
            # grid[cerrent_po[0]+1][cerrent_po[1]][3] = 1

            cerrent_po = [cerrent_po[0],cerrent_po[1]+1]

        if status_logic == 'turn left':
            grid[cerrent_po[0]][cerrent_po[1]][1] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have left wall 
            grid[cerrent_po[0]][cerrent_po[1]][0] = 2 #grid[cerrent_po[0]][cerrent_po[1]] have front wall 
            grid[cerrent_po[0]+1][cerrent_po[1]][4] = 1
            # grid[cerrent_po[0]+1][cerrent_po[1]][3] = 1 
            cerrent_po = [cerrent_po[0]+1,cerrent_po[1]]


def logic():
    if status_tof == False and status_ss_r == True:
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        time.sleep(0.2)
                    
        move_forward()
        return 'move_forward'
                
    elif status_ss_r == False:            
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        time.sleep(0.2)
        adjust_wall()
        turn_right()
        adjust_wall_l()
        adjust_wall_f()
        move_forward()
        return 'turn right'

    elif status_tof == True and status_ss_r == True and status_ss_l == True:
        ep_gimbal.moveto(pitch=0, yaw=-90, pitch_speed=100, yaw_speed=30).wait_for_completed()
        tof_check_left = tof_distance
        if tof_check_left < 400:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            time.sleep(0.2)
            adjust_wall()
            turn_back()
            adjust_wall_r()
            adjust_wall_f()
            move_forward()
            return 'turn back'
                    
        else:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            time.sleep(0.2)
            adjust_wall()
            turn_left()
            adjust_wall_r()
            adjust_wall_f()
            move_forward()
            return 'turn left'
                        

    elif status_tof == True and status_ss_r == True:
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        time.sleep(0.2)
        adjust_wall()
        turn_left()
        adjust_wall_r()
        adjust_wall_f()
        move_forward()
        return 'turn left'


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
    # เริ่มต้นการทำงานของหุ่นยนต์
    ep_robot = robot.Robot()
    print("Initializing robot...")
    ep_robot.initialize(conn_type="ap")
    time.sleep(2)  # รอ 2 วินาทีหลังการเชื่อมต่อ

    # เริ่มต้นการทำงานของเซ็นเซอร์ต่าง ๆ
    ep_sensor = ep_robot.sensor
    ep_chassis = ep_robot.chassis
    ep_gimbal = ep_robot.gimbal
    ep_sensor_adaptor = ep_robot.sensor_adaptor

    # สมัครสมาชิกฟังก์ชัน callback เพื่อรับข้อมูลจากเซ็นเซอร์ ToF และ Sharp Sensors
    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_sensor.sub_distance(freq=10, callback=tof_data_handler)
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)

    # ep_sensor_adaptor.sub_adapter(freq=5, callback=sub_data_handler)


    # ปรับตำแหน่ง Gimbal ให้ตรงศูนย์
    ep_gimbal.recenter(pitch_speed=400, yaw_speed=400).wait_for_completed()
    adjust_wall()
    

    try:
        while True:
            if tof_distance is None or adc_l is None or adc_r is None:
                print("Waiting for sensor data...")
                time.sleep(1)
                continue

            maze_complete = False
            while not maze_complete:
                update_wall()
                print_pretty_grid(grid)
                
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
        print("Cleaning up...")
        ep_chassis.unsub_position()
        ep_chassis.unsub_attitude()
        ep_sensor.unsub_distance()
        ep_chassis.drive_speed(x=0, y=0, z=0)
        ep_robot.close()
        print("Program ended.")