import robomaster
from robomaster import robot
import time

left_data = []
right_data = []
left_time_data = []
right_time_data = []

def sub_data_handler(sub_info):
    io_data, ad_data = sub_info
    
    # Convert each ADC value to voltage and calculate distance
    distances = []
    for adc_value in ad_data:
        voltage = adc_value * 3.3 / 1023

        # Define the piecewise linear approximation ranges and coefficients
        ranges = [
            {'m': -0.3846, 'c': 4.30764, 'min': 2.2, 'max': 3.2},
            {'m': -0.2, 'c': 3.2, 'min': 1.4, 'max': 2.2},
            {'m': -0.067, 'c': 1.87, 'min': 0.8, 'max': 1.4},
            {'m': -0.034, 'c': 1.344, 'min': 0.4, 'max': 0.8}
        ]

        distance = None
        for range_ in ranges:
            if range_['min'] <= voltage < range_['max']:
                distance = (voltage - range_['c']) / range_['m']
                distances.append(distance)
                break
    
    # Calculate avg for left and right sensor
    left = sum(distances[0:2]) / 2
    right = sum(distances[2:4]) / 2
    left_data.append(left)
    right_data.append(right)
    left_time_data.append(time.time())
    right_time_data.append(time.time())

    # print(f"io value: {io_data}, ad values: {ad_data}")
    # print(f"Distance to the foam wall: {distances} cm")
    print(f"port1 left: {left}, port2 right: {right}")
    return distances


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_sensor = ep_robot.sensor_adaptor
    ep_sensor.sub_adapter(freq=5, callback=sub_data_handler)
    time.sleep(5)
    ep_sensor.unsub_adapter()
    ep_robot.close()