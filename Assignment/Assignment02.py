import cv2
import robomaster
from robomaster import robot, camera
import time
import numpy as np


def sub_position_handler(position_info):
    global current_x
    current_x, y, z = position_info


def detect_chicken():
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

    center_x = 1280 / 2
    center_y = 720 / 2

    current_x = 0.0  
    target_distance = 1.3
    kp, ki, kd = 75, 10, 30   
    tolerance = 0.01   

    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    ep_gimbal.moveto(pitch=-10, yaw=0).wait_for_completed()

    overall_start_time = time.time()
    prev_error = 0.0
    integral = 0.0
    prev_time = overall_start_time


    while True:
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_chicken = np.array([33, 150, 100])
        upper_chicken = np.array([38, 255, 255])

        lower_bottle = np.array([95, 80, 100])
        upper_bottle = np.array([120, 255, 255])

        mask_chicken = cv2.inRange(hsv_frame, lower_chicken, upper_chicken)
        mask_bottle = cv2.inRange(hsv_frame, lower_bottle, upper_bottle)

        contours_chicken, _ = cv2.findContours(mask_chicken.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_bottle, _ = cv2.findContours(mask_bottle.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours_chicken: detect_chicken()
        if contours_bottle: detect_bottle()

        if target_distance - current_x > tolerance :
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
            time.sleep(0.005)  

        else:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            time.sleep(0.005)
                
        cv2.imshow("Original Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):break

        time.sleep(0.1)

    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_chassis.unsub_position()
    ep_robot.close()