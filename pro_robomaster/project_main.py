import robomaster
from robomaster import robot
import math
import matplotlib.pyplot as plt

from project_functions import *
import pandas as pd

ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")

ep_sensor_adaptor = ep_robot.sensor_adaptor
ep_chassis = ep_robot.chassis
ep_gimbal = ep_robot.gimbal
ep_battery = ep_robot.battery
ep_sensor = ep_robot.sensor
state = {"front": {"front": True, "left": True, "right": True, "back": True}}

map_list = []
map_way = []
if __name__ == "__main__":
    ep_sensor.sub_distance(freq=5, callback=get_infared_distance_sensor_buidin)
    ep_chassis.sub_position(freq=5, callback=sub_position_handler)
    ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=210).wait_for_completed()
    time.sleep(0.5)
    start = time.time()
    for i in range(50):
        print(i)
        direction_distance, state = check_all_side(
            ep_gimbal, ep_chassis, ep_sensor, ep_sensor_adaptor, state
        )

        x, y = get_sub_position_now()
        x, y = get_x_y_graph_map(x, y)
        state_x_y = {f"{x+6} {y}": None}
        for s, t in zip(state_x_y, state):
            state_x_y[s] = state[t]
        map_way.append(state_x_y)
        print(x, y)
        map_list.append((x, y + 6))

        if direction_distance["left"] > 50:
            state = check_way(
                move_left_tile(
                    range=1,
                    ep_gimbal=ep_gimbal,
                    ep_chassis=ep_chassis,
                    ep_sensor_adaptor=ep_sensor_adaptor,
                ),
                state,
            )
        elif direction_distance["front"] > 50:
            state = check_way(
                move_forward_tile(
                    range=1,
                    ep_gimbal=ep_gimbal,
                    ep_chassis=ep_chassis,
                    ep_sensor_adaptor=ep_sensor_adaptor,
                ),
                state,
            )
        elif direction_distance["right"] > 50:
            state = check_way(
                move_right_tile(
                    range=1,
                    ep_gimbal=ep_gimbal,
                    ep_chassis=ep_chassis,
                    ep_sensor_adaptor=ep_sensor_adaptor,
                ),
                state,
            )
        else:
            state = check_way(
                move_back_tile(
                    range=1,
                    ep_gimbal=ep_gimbal,
                    ep_chassis=ep_chassis,
                    ep_sensor_adaptor=ep_sensor_adaptor,
                ),
                state,
            )

        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

        to_center_of_tile(ep_gimbal, ep_chassis, ep_sensor_adaptor)

        end = time.time()

        print("---------This Time ", end - start)
        if end - start >= 240:
            break

    ep_robot.close()

    print("map_lst:", map_list)
    with open("output2.txt", "w") as file:
        for item in map_list:
            print(item)
            file.write(f"{item[0]},{item[1]}\n")

    plt.plot([i[1] for i in map_list], [i[0] for i in map_list])
    plt.show()

    print(map_list)
    print(map_way)
