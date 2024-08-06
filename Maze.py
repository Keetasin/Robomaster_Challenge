import robomaster
from robomaster import robot
import time

# 260-400
def sub_data_handler_left_right(sub_info):
    io_data, ad_data = sub_info
    left = sum(ad_data[0:2])/2
    right = sum(ad_data[2:4])/2
    print(f"port1 left: {left}, port2 right: {right}")
    
    
def sub_data_handler_forword(sub_info):
    distance = sub_info
    print(f"tof forword:{distance[0]}")


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="rndis")

    ep_sensor1 = ep_robot.sensor_adaptor
    ep_sensor2 = ep_robot.sensor
    ep_sensor1.sub_adapter(freq=5, callback=sub_data_handler_left_right)
    ep_sensor2.sub_distance(freq=5, callback=sub_data_handler_forword)
    time.sleep(5)
    ep_sensor1.unsub_adapter()
    ep_sensor2.unsub_distance()
    ep_robot.close()