import robomaster
import time
from robomaster import robot

import math

# กำหนด ค่าข้อมูลที่ได้จากการอ่านค่า
dij_infared_distance_sensor_value = 0
global_x_sub_position = 0
global_y_sub_position = 0
global_z_sub_position = 0

# กำหนดความเร็วโดยทั่วไป
speed = 0.3
# กำหนดว่าหันไปทางใดแล้วตอนนี้ ทิศใดของรถสามารถเคลื่อนทีไปได้
state = {"front": {"front": True, "left": True, "right": True, "back": True}}


# เป็นการอ่านค่าองศาตำแหน่งของหัวของรถ
def get_angle(angle_info):
    pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle = angle_info
    print(
        "gimbal angle: pitch_angle:{0}, yaw_angle:{1}, pitch_ground_angle:{2}, yaw_ground_angle:{3}".format(
            pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle
        )
    )


# เป็นการอ่านค่าของการของตัวเซนเซอร์ตัวนอกที่นำมาใช้คือตัว Analog Distance Sensor (Sharp)
def get_distance_from_ir_sensor_external(adc):
    # volt = (3.3 / 1024 * adc) + 0.05
    # re_distance = volt / 12.8
    # distace = 1 / re_distance
    if not adc:
        return 20
    # เข้ากระบวนการแปลงข้อมูลทีได้ออกมาเป็นระยะทาง เซนติเมตร
    distace = 1 / (((3.3 / 1024 * adc) + 0.05) / 12.8)
    if distace > 20:
        return distace
    return distace


# กำหนดการอ่านค่าจากเซนเซอร์โดยกำหนด id และ port ตรงกับที่ได้ต่อไว้
def get_left_ir_sensor(ep_sensor_adaptor):
    return (
        sum(
            [
                get_distance_from_ir_sensor_external(
                    ep_sensor_adaptor.get_adc(id=3, port=1)
                    # ep_sensor_adaptor.get_adc(id=2, port=1)
                )
                for i in range(10)
                if i != None
            ]
        )
        / 10
    )


# กำหนดการอ่านค่าจากเซนเซอร์โดยกำหนด id และ port ตรงกับที่ได้ต่อไว้
def get_right_ir_sensor(ep_sensor_adaptor):
    return (
        sum(
            [
                get_distance_from_ir_sensor_external(
                    ep_sensor_adaptor.get_adc(id=2, port=2)
                    # ep_sensor_adaptor.get_adc(id=1, port=2)
                )
                for i in range(10)
            ]
        )
        / 10
    )


# เป็นการข้อมูลตำแหน่งของรถว่ามีการเคลื่อนไปในทิศทางใดเคลื่อนที่ไปเท่าไร
def sub_position_handler(position_info):
    x, y, z = position_info
    global global_x_sub_position
    global global_y_sub_position
    # global global_z_sub_position
    global_x_sub_position = x
    global_y_sub_position = y
    # global_z_sub_position = z
    # print("chassis position: x:{0}, y:{1}, z:{2}".format(x, y, z))


# ทำการอ่านค่าของตำแหน่งในขณะนี้
def get_sub_position_now():
    x_position = global_x_sub_position
    y_position = global_y_sub_position
    print("chassis position: x:{0}, y:{1}".format(x_position, y_position))

    return x_position, y_position


# ทำการนำข้อมูลของ x y มาหารเพื่อทำการพล็อตกราฟ
def get_x_y_graph_map(x, y):
    x = float("{:.0f}".format(x / 0.6))
    y = float("{:.0f}".format(y / 0.6))
    return x, y


# เป็นการอ่านค่าข้อมูลแบตเตอรี่ว่ามีเท่าไร
def get_battery(batter_info, ep_robot):
    percent = batter_info
    # print("Battery: {0}%.".format(percent))
    ep_led = ep_robot.led
    brightness = int(percent * 255 / 100)
    ep_led.set_led(comp="all", r=brightness, g=brightness, b=brightness)


# เป็นการอ่านตัวเซนเซอร์ภายใน คือเซนเซอร์ที่ติดอยู่บนหัวของรถ ว่ามีการอ่านค่าได้เท่าไร
def get_infared_distance_sensor_buidin(sub_info):
    d = sub_info
    global dij_infared_distance_sensor_value
    average = []
    for i in range(5):
        average.append((d[0] / (2 * math.tan(10))) / 10)
    dij_infared_distance_sensor_value = sum(average) / 5
    # print(
    #     "tof1:{0}  tof2:{1}  tof3:{2}  tof4:{3}".format(
    #         distance[0], distance[1], distance[2], distance[3]
    #     )
    # )
    # print("Front Distance : ", dij_infared_distance_sensor_value, "cm.")  # cm
    # print(dij_infared_distance_sensor_value)


# ส่งข้อมูลระยะทางที่อ่านของเซนเซอร์ของบนหัวรถ
def get_dis():
    distance = dij_infared_distance_sensor_value
    return distance


# เป็นการตรวจสอบระยะทางของทุกทิศทางว่ามีระยะทางเท่าไร
def check_all_side(ep_gimbal, ep_chassis, ep_sensor, ep_sensor_adaptor, state):
    ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=270).wait_for_completed()
    direction = {"front": 0, "left": -90, "right": 90, "back": 180}
    direction_distance = {"back": 0}
    for direc in direction.keys():
        # if direc == "front":
        #     # time.sleep(0.2)
        #     direction_distance[direc] = dij_infared_distance_sensor_value
        #     # time.sleep(0.2)

        if direc != "back":
            ep_gimbal.moveto(
                pitch=0, yaw=direction[direc], pitch_speed=50, yaw_speed=270
            ).wait_for_completed()
            time.sleep(0.2)
            direction_distance[direc] = dij_infared_distance_sensor_value
            time.sleep(0.05)
        # elif direc == "left":
        #     direction_distance[direc] = get_left_ir_sensor(ep_sensor_adaptor)
        # elif direc == "right":
        #     direction_distance[direc] = get_right_ir_sensor(ep_sensor_adaptor)
        else:
            pass

    for key in state:
        for i, j in zip(direction_distance, state[key]):
            if direction_distance[i] <= 50:
                state[key][j] = False
            else:
                state[key][j] = True
    ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=210).wait_for_completed()
    return direction_distance, state


# เป็นทำเงื่อนไขว่าทิศทางใดสามารถไปได้บ้าง
def check_way(data, state):
    for i in state.keys():
        if i == "front":
            if data == "front":
                state = {
                    "front": {"front": True, "left": True, "right": True, "back": True}
                }
            if data == "left":
                state = {
                    "left": {"left": True, "back": True, "front": True, "right": True}
                }
            if data == "right":
                state = {
                    "right": {"right": True, "front": True, "back": True, "left": True}
                }

            if data == "back":
                state = {
                    "back": {"back": True, "right": True, "left": True, "front": True}
                }

        elif i == "left":
            if data == "front":
                state = {
                    "left": {"left": True, "back": True, "front": True, "right": True}
                }
            elif data == "left":
                state = {
                    "back": {"back": True, "right": True, "left": True, "front": True}
                }
            elif data == "right":
                state = {
                    "front": {"front": True, "left": True, "right": True, "back": True}
                }
            elif data == "back":
                state = {
                    "right": {"right": True, "front": True, "back": True, "left": True}
                }
        elif i == "right":
            if data == "front":
                state = {
                    "right": {"right": True, "front": True, "back": True, "left": True}
                }
            elif data == "left":
                state = {
                    "front": {"front": True, "left": True, "right": True, "back": True}
                }
            elif data == "right":
                state = {
                    "back": {"back": True, "right": True, "left": True, "front": True}
                }

            elif data == "back":
                state = {
                    "left": {"left": True, "back": True, "front": True, "right": True}
                }

        elif i == "back":
            if data == "front":
                state = {
                    "back": {"back": True, "right": True, "left": True, "front": True}
                }
            elif data == "left":
                state = {
                    "right": {"right": True, "front": True, "back": True, "left": True}
                }
            elif data == "right":
                state = {
                    "left": {"left": True, "back": True, "front": True, "right": True}
                }

            elif data == "back":
                state = {
                    "front": {"front": True, "left": True, "right": True, "back": True}
                }
    return state


# เป็นการเคลื่อนที่ไปข้างหลังเรื่อยๆจนกว่าจะเจอกำแพงในระยะเท่าไร
def move_back_until(range, ep_gimbal, ep_chassis, ep_sensor_adaptor):
    ep_gimbal.moveto(
        pitch=0, yaw=180, pitch_speed=50, yaw_speed=90
    ).wait_for_completed()
    distance = dij_infared_distance_sensor_value

    while distance > range:
        # print("Back Distance : ", distance, "cm.")
        ep_chassis.drive_speed(x=-speed, y=0, z=0, timeout=1)
        distance = dij_infared_distance_sensor_value

    print("end")

    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=90).wait_for_completed()


# เป็นการเคลื่อนที่ไปข้างหน้าเรื่อยๆจนกว่าจะเจอกำแพงในระยะเท่าไร
def move_forward_until(range, ep_gimbal, ep_chassis, ep_sensor_adaptor):
    # ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=90).wait_for_completed()
    distance = dij_infared_distance_sensor_value

    while distance > range:
        # print("Forward Distance : ", distance, "cm.")
        ep_chassis.drive_speed(x=speed, y=0, z=0, timeout=1)
        distance = dij_infared_distance_sensor_value
    print("end")

    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    # ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=90).wait_for_completed()


# เป็นการเคลื่อนที่ไปทางซ้ายเรื่อยๆจนกว่าจะเจอกำแพงในระยะเท่าไร
def move_left_until(range, ep_gimbal, ep_chassis, ep_sensor_adaptor):
    ep_gimbal.moveto(
        pitch=0, yaw=-90, pitch_speed=50, yaw_speed=90
    ).wait_for_completed()

    distance = get_left_ir_sensor(ep_sensor_adaptor)

    while distance > range:
        # print("Left Distance : ", distance, "cm.")
        ep_chassis.drive_speed(x=0, y=-speed, z=0, timeout=1)
        distance = get_left_ir_sensor(ep_sensor_adaptor)
    print("end")

    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=90).wait_for_completed()


# เป็นการเคลื่อนที่ไปทางขวาเรื่อยๆจนกว่าจะเจอกำแพงในระยะเท่าไร
def move_right_until(range, ep_gimbal, ep_chassis, ep_sensor_adaptor):
    ep_gimbal.moveto(pitch=0, yaw=90, pitch_speed=50, yaw_speed=90).wait_for_completed()

    distance = get_right_ir_sensor(ep_sensor_adaptor)

    while distance > range:
        # print("Right Distance : ", distance, "cm.")
        ep_chassis.drive_speed(x=0, y=speed, z=0, timeout=1)
        distance = get_right_ir_sensor(ep_sensor_adaptor)
    print("end")

    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=90).wait_for_completed()


# เป็นทดสอบว่าหากเคลื่อนที่เป็นรูปสี่เหลี่ยมจะเจอความคลาดเคลื่อนมากเท่าไร
def square_way(ep_gimbal, ep_chassis, ep_sensor_adaptor):
    move_forward_until(
        range=30,
        ep_gimbal=ep_gimbal,
        ep_chassis=ep_chassis,
        ep_sensor_adaptor=ep_sensor_adaptor,
    )
    ep_chassis.move(x=0, y=0, z=-7, xy_speed=0, z_speed=90).wait_for_completed()

    move_left_until(
        range=15,
        ep_gimbal=ep_gimbal,
        ep_chassis=ep_chassis,
        ep_sensor_adaptor=ep_sensor_adaptor,
    )
    # ep_chassis.move(x=0, y=0, z=-7, xy_speed=0, z_speed=90).wait_for_completed()

    move_back_until(
        range=30,
        ep_gimbal=ep_gimbal,
        ep_chassis=ep_chassis,
        ep_sensor_adaptor=ep_sensor_adaptor,
    )
    ep_chassis.move(x=0, y=0, z=-7, xy_speed=0, z_speed=90).wait_for_completed()

    move_right_until(
        range=15,
        ep_gimbal=ep_gimbal,
        ep_chassis=ep_chassis,
        ep_sensor_adaptor=ep_sensor_adaptor,
    )
    ep_chassis.move(x=0, y=0, z=-7, xy_speed=0, z_speed=90).wait_for_completed()


# กำหนดว่าเคลื่อนที่ 1 กระเบื้อง จะต้องเคลือ่นที่ไปเท่าไร
def move_one_tile(ep_chassis):
    print("Move 1 Tile")
    ep_chassis.move(x=0.58, y=0, z=0, xy_speed=2, z_speed=0).wait_for_completed()

    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)


# เป็นการตรวจสอบว่า มีระยะทางข้างหน้าเพียงพอให้เคลื่อนที่ไป 1 กระเบื้องหรือไม่
def check_distance_for_move():
    distance = dij_infared_distance_sensor_value
    if distance <= 60:
        print(f"{distance} Not Enough Distance")
        return True
    print(f"{distance} Can Move")
    return False


# ทำการสั่งให้เคลื่อนที่ไป 1 กระเบื้องไปข้างหน้า
def move_forward_tile(range, ep_gimbal, ep_chassis, ep_sensor_adaptor):
    print("Move Forward Tile")
    ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=270).wait_for_completed()
    distance = dij_infared_distance_sensor_value
    tile = 0
    while tile < range:
        # print("Forward Distance : ", distance, "cm.")
        if check_distance_for_move():
            break
        move_one_tile(ep_chassis)
        tile += 1
    # print("end")
    # ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    # ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=90).wait_for_completed()
    return "front"


# ทำการสั่งให้เคลื่อนที่ไป 1 กระเบื้องไปข้างหลัง
def move_back_tile(range, ep_gimbal, ep_chassis, ep_sensor_adaptor):
    print("Move Back Tile")
    ep_chassis.move(x=0, y=0, z=177, xy_speed=0, z_speed=110).wait_for_completed()
    ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=270).wait_for_completed()
    distance = dij_infared_distance_sensor_value
    tile = 0
    while tile < range:
        # print("Forward Distance : ", distance, "cm.")
        if check_distance_for_move():
            break
        move_one_tile(ep_chassis)
        tile += 1
    # print("end")

    # ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    # ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=90).wait_for_completed()
    return "back"


# ทำการสั่งให้เคลื่อนที่ไป 1 กระเบื้องไปทางซ้าย
def move_left_tile(range, ep_gimbal, ep_chassis, ep_sensor_adaptor):
    print("Move Left Tile")
    ep_chassis.move(x=0, y=0, z=90, xy_speed=0, z_speed=90).wait_for_completed()
    ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=270).wait_for_completed()
    distance = dij_infared_distance_sensor_value
    tile = 0
    while tile < range:
        # print("Forward Distance : ", distance, "cm.")
        if check_distance_for_move():
            break
        move_one_tile(ep_chassis)
        tile += 1
    # print("end")

    # ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    # ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=90).wait_for_completed()
    return "left"


# ทำการสั่งให้เคลื่อนที่ไป 1 กระเบื้องไปทางขวา
def move_right_tile(range, ep_gimbal, ep_chassis, ep_sensor_adaptor):
    print("Move Right Tile")
    ep_chassis.move(x=0, y=0, z=-90, xy_speed=0, z_speed=90).wait_for_completed()
    ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=270).wait_for_completed()
    distance = dij_infared_distance_sensor_value
    tile = 0
    while tile < range:
        # print("Forward Distance : ", distance, "cm.")
        if check_distance_for_move():
            break
        move_one_tile(ep_chassis)
        tile += 1
    # print("end")

    # ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    # ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=90).wait_for_completed()
    return "right"


# เป็นกำหนดให้เคลื่อนที่ขยับตัวรถให้อยู่ในส่วนประมาณของกลางแผ่นกระเบื้อง
def to_center_of_tile(ep_gimbal, ep_chassis, ep_sensor_adaptor):
    time.sleep(0.1)
    distance = dij_infared_distance_sensor_value
    print("Front ", distance)
    if distance <= 50:
        if distance >= 40:
            ep_chassis.move(x=0.2, y=0, z=0, xy_speed=1, z_speed=0).wait_for_completed()
        elif distance >= 25:
            ep_chassis.move(x=0.1, y=0, z=0, xy_speed=1, z_speed=0).wait_for_completed()
        elif distance <= 15:
            ep_chassis.move(
                x=-0.12, y=0, z=0, xy_speed=1, z_speed=0
            ).wait_for_completed()
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    left_distance = get_left_ir_sensor(ep_sensor_adaptor)
    print("Left ", left_distance)
    if left_distance <= 25:
        if left_distance >= 15:
            ep_chassis.move(
                x=0, y=-0.05, z=0, xy_speed=1, z_speed=0
            ).wait_for_completed()
        elif left_distance >= 15:
            ep_chassis.move(
                x=0, y=-0.05, z=0, xy_speed=1, z_speed=0
            ).wait_for_completed()
        elif left_distance <= 7:
            ep_chassis.move(x=0, y=0.1, z=0, xy_speed=1, z_speed=0).wait_for_completed()
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    right_distance = get_right_ir_sensor(ep_sensor_adaptor)
    print("Right ", right_distance)

    if right_distance <= 7:
        ep_chassis.move(x=0, y=-0.05, z=0, xy_speed=1, z_speed=0).wait_for_completed()

    # print(
    #     "Left Distance",
    #     get_left_ir_sensor(ep_sensor_adaptor),
    #     "Right Distance",
    #     get_right_ir_sensor(ep_sensor_adaptor),
    # )
    # print("Complete Center")
