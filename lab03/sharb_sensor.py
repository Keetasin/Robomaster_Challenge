import robomaster
from robomaster import robot
import time

left_data = []
right_data = []
left_time_data = []
right_time_data = []


def sub_data_handler(sub_info):
    io_data, ad_data = sub_info
    
    distances = []
    for adc_value in ad_data:
        voltage = adc_value * 3.3 / 1023
        # print(voltage)
        # Adjusted piecewise linear approximation
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
                distance = 50.0

        distances.append(distance)

    left = sum(distances[0:2]) / 2
    right = sum(distances[2:4]) / 2
    left_data.append(left)
    right_data.append(right)
    left_time_data.append(time.time())
    right_time_data.append(time.time())
    
    if 9 <= left <= 13:
        left += 1
    elif 14 <= left <= 16:  
        left += 3

    if left >= 25:
        left = 50
    if right >= 25:
        right = 50
    if distance == 0:
        print("!" * 15)
        
    # print("--> ad_data", ad_data[0:4])
    print(f"port1 left: {left}, port2 right: {right}")

    return distances

def sub_tof_handler(sub_info):
    distance = sub_info
    print("tof1:{0}".format(distance[0]))


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_gimbal = ep_robot.gimbal
    ep_gimbal.recenter().wait_for_completed()

    ep_sensor = ep_robot.sensor
    ep_sensor_adaptor = ep_robot.sensor_adaptor

    ep_sensor.sub_distance(freq=5, callback=sub_tof_handler)
    ep_sensor_adaptor.sub_adapter(freq=5, callback=sub_data_handler)
    time.sleep(20)
    ep_sensor.unsub_adapter()
    ep_sensor.unsub_distance()

    ep_robot.close()
