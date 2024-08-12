from robomaster import robot
import time

sharp_left_data = []
sharp_right_data = []

def filter_adc_value(ad_data):
    y_filtered = []
    alpha = 0.1  # ตั้งค่า alpha ตามความต้องการ
    y_prev = 0  # ค่าเริ่มต้นของ y[n-1]

    for adc_value in ad_data: # 153, 153, 203, 203
        y_current = alpha * y_prev + (1 - alpha) * adc_value
        y_filtered.append(y_current)
        y_prev = y_current

    return y_filtered # 150, 150, 200, 200

def sub_data_handler(sub_info):
    io_data, ad_data = sub_info
    
    # กรองค่า ADC
    y_filtered = filter_adc_value(ad_data)

    distances = []
    for adc_filtered in  y_filtered:
        voltage = adc_filtered * 3.3 / 1023

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
                distance = (voltage - 1.344) / -0.034

        distances.append(distance)
    
    sharp_left = sum(distances[0:2]) / 2
    sharp_right = sum(distances[2:4]) / 2
    sharp_left_data.append(sharp_left)
    sharp_right_data.append(sharp_right)
    
    # Error left 
    if sharp_left >= 13:
        sharp_left += 2
    
    # ตัดช่วง
    if sharp_left >= 20:
        sharp_left = 50
    if sharp_right >= 26:
        sharp_right = 50

    print(f"PORT1 left: {sharp_left}, PORT2 right: {sharp_right}")
    
    return distances

def sub_tof_handler(sub_info):
    distance = sub_info
    # print("tof1:{0}".format(distance[0]))


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_gimbal = ep_robot.gimbal
    ep_gimbal.recenter().wait_for_completed()

    ep_sensor = ep_robot.sensor
    ep_sensor_adaptor = ep_robot.sensor_adaptor

    ep_sensor.sub_distance(freq=5, callback=sub_tof_handler)
    ep_sensor_adaptor.sub_adapter(freq=5, callback=sub_data_handler)

    time.sleep(60)

    ep_sensor.unsub_distance()
    ep_sensor_adaptor.unsub_adapter()

    ep_robot.close()



