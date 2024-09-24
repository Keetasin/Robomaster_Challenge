import cv2
import robomaster
from robomaster import robot, camera, blaster
import time
import numpy as np

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
    

def sub_position_handler(position_info):
    global current_x
    current_x, y, z = position_info
    # print(current_x)

def detect_chicken(contours_chicken):
    max_chicken_contour = max(contours_chicken, key=cv2.contourArea)
    (x_chicken, y_chicken, w_chicken, h_chicken) = cv2.boundingRect(max_chicken_contour)
    
    if w_chicken > 65:
        add_w_chicken = int(w_chicken * 0.3)
        add_h_chicken = int(h_chicken * 0.6)
        new_y_chicken = y_chicken - add_h_chicken // 2 + 20
    elif w_chicken > 25:
        add_w_chicken = int(w_chicken * 0.35)
        add_h_chicken = int(h_chicken * 0.55)
        new_y_chicken = (y_chicken - add_h_chicken // 2 + 20) - 15
    else:
        add_w_chicken = int(w_chicken * 0.39)
        add_h_chicken = int(h_chicken * 0.59)
        new_y_chicken = (y_chicken - add_h_chicken // 2 + 20) - 15

    new_x_chicken = x_chicken - add_w_chicken // 2
    new_w_chicken = w_chicken + add_w_chicken
    new_h_chicken = h_chicken + add_h_chicken
    cv2.rectangle(frame, (new_x_chicken, new_y_chicken), (new_x_chicken + new_w_chicken, new_y_chicken + new_h_chicken), (255, 0, 255), 2)
    cv2.putText(frame, f" chicken x: {x_chicken}, y: {y_chicken}, w: {w_chicken}, h: {h_chicken}", (x_chicken, y_chicken - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    return Marker(new_x_chicken, new_y_chicken, new_w_chicken, new_h_chicken)


def detect_bottle():
    max_bottle_contour = max(contours_bottle, key=cv2.contourArea)
    (x_bottle, y_bottle, w_bottle, h_bottle) = cv2.boundingRect(max_bottle_contour)

    if w_bottle > 85:   
        add_w_bottle = int(w_bottle * 0.27)  
        add_h_bottle = int(h_bottle * 2.7)
        new_y_bottle = y_bottle - add_h_bottle // 2 -13

    elif w_bottle > 50:  
        add_w_bottle = int(w_bottle * 0.27)  
        add_h_bottle = int(h_bottle * 2.9)
        new_y_bottle = y_bottle - add_h_bottle // 2 -11    

    elif w_bottle > 35:  
        add_w_bottle = int(w_bottle * 0.28)  
        add_h_bottle = int(h_bottle * 3.3)
        new_y_bottle = y_bottle - add_h_bottle // 2 -4

    else:
        add_w_bottle = int(w_bottle * 0.47)
        add_h_bottle = int(h_bottle * 3.4)
        new_y_bottle = y_bottle - add_h_bottle // 2 -4

    new_x_bottle = x_bottle - add_w_bottle // 2
    new_w_bottle = w_bottle + add_w_bottle
    new_h_bottle = h_bottle + add_h_bottle

    cv2.rectangle(frame, (new_x_bottle, new_y_bottle), (new_x_bottle + new_w_bottle, new_y_bottle + new_h_bottle), (0, 165, 255), 2)
    cv2.putText(frame, f" BOTTLE x: {x_bottle}, y: {y_bottle}, w: {w_bottle}, h: {h_bottle}", (x_bottle, y_bottle-50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_chassis = ep_robot.chassis
    ep_blaster = ep_robot.blaster

    center_x = 1280 / 2
    center_y = 720 / 2

    current_x = 0.0  
    target_distance = 1.3
    kp, ki, kd = 75, 10, 30   
    tolerance = 0.01   
    prev_error, integral = 0.0, 0.0
    prev_time = time.time()

    p = 0.6 / 1.7
    i = p / (0.7 / 2)
    d = p * (0.7 / 8)

    accumulate_err_x = 0
    accumulate_err_y = 0
    data_pitch_yaw = []
    prev_time = time.time()

    xx = True
    count = 0
    

    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_gimbal.sub_angle(freq=10)
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()

    while True:
        # [250:600,300:980]
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_chicken = np.array([33, 150, 100])
        upper_chicken = np.array([38, 255, 255])

        lower_bottle = np.array([95, 80, 100])
        upper_bottle = np.array([120, 255, 255])

        mask_chicken = cv2.inRange(hsv_frame, lower_chicken, upper_chicken)
        mask_bottle = cv2.inRange(hsv_frame, lower_bottle, upper_bottle)

        contours_chicken, _ = cv2.findContours(mask_chicken, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_bottle, _ = cv2.findContours(mask_bottle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours_chicken: marker = detect_chicken(contours_chicken)
        if contours_bottle: detect_bottle()
        if marker._w >65 and count == 0: xx = False
        if count > 0 :count += 1
        if count == 50:  count = 0
        print(count)
        
        if target_distance - current_x > tolerance and xx:
            current_time = time.time()
            error = target_distance - current_x
            time_diff = current_time - prev_time
            integral += error * time_diff
            derivative = (error - prev_error) / time_diff if time_diff > 0 else 0.0
            speed = kp * error + kd * derivative + ki * integral 
            speed = max(min(speed, 20), 0)  
            ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
            prev_error = error
            prev_time = current_time
            time.sleep(0.05)  

        else:
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

                
                # cv2.putText(frame, marker.text, marker.center, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                
                if  marker._w >=65:
                    cv2.rectangle(frame, marker.pt1, marker.pt2, (0, 0, 0), 2)
                    ep_blaster.fire(fire_type=blaster.INFRARED_FIRE, times=1)
                    xx = True
                    count += 1
                    print('---------')
                    

            else:
                ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)

        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        time.sleep(0.1)

    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_chassis.unsub_position()
    ep_robot.close()
