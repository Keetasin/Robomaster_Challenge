import cv2
import robomaster
from robomaster import robot, camera
import time
import numpy as np


def sub_position_handler(position_info):
    global current_x
    x, y, z = position_info
    current_x = x
    print("position: x:{0}, y:{1}, z:{2}".format(x, y, z))

def chick_detect():
    if contours_chick:
        max_chick_contour = max(contours_chick, key=cv2.contourArea)
        (x_chick, y_chick, w_chick, h_chick) = cv2.boundingRect(max_chick_contour)

        if w_chick > 65:
            add_w_chick = int(w_chick * 0.3)
            add_h_chick = int(h_chick * 0.6)
            # ปรับตำแหน่งของกรอบให้เลื่อนไปทางซ้ายและขึ้นข้างบน
            new_x_chick = x_chick - add_w_chick // 2
            new_y_chick = y_chick - add_h_chick // 2 + 20
            new_w_chick = w_chick + add_w_chick
            new_h_chick = h_chick + add_h_chick
        else:
            add_w_chick = int(w_chick * 0.3)
            add_h_chick = int(h_chick * 0.5)
            # ปรับตำแหน่งของกรอบให้เลื่อนไปทางซ้ายและขึ้นข้างบน
            new_x_chick = x_chick - add_w_chick // 2
            new_y_chick = (y_chick - add_h_chick // 2 + 20) - 15
            new_w_chick = w_chick + add_w_chick
            new_h_chick = h_chick + add_h_chick
        cv2.rectangle(
            frame,
            (new_x_chick, new_y_chick),
            (new_x_chick + new_w_chick, new_y_chick + new_h_chick),
            (0, 0, 255),
            2,
        )

        text_chick = f" CHICK x: {x_chick}, y: {y_chick}, w: {w_chick}, h: {h_chick}"
        cv2.putText(
            frame,
            text_chick,
            (x_chick, y_chick - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
    else:
        pass


def bottle_detect():
    if contours_bottle:
        max_bottle_contour = max(contours_bottle, key=cv2.contourArea)
        (x_bottle, y_bottle, w_bottle, h_bottle) = cv2.boundingRect(max_bottle_contour)

        add_w_bottle = int(w_bottle * 1.3)
        add_h_bottle = int(h_bottle * 4)

        new_x_bottle = x_bottle - add_w_bottle // 2
        new_y_bottle = y_bottle - add_h_bottle // 2 - 5
        new_w_bottle = w_bottle + add_w_bottle
        new_h_bottle = h_bottle + add_h_bottle

        cv2.rectangle(
            frame,
            (new_x_bottle, new_y_bottle),
            (new_x_bottle + new_w_bottle, new_y_bottle + new_h_bottle),
            (255, 0, 0),
            2,
        )

        text_bottle = (
            f" BOTTLE x: {x_bottle}, y: {y_bottle}, w: {w_bottle}, h: {h_bottle}"
        )
        cv2.putText(
            frame,
            text_bottle,
            (x_bottle, y_bottle - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
    else:
        pass



if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_chassis = ep_robot.chassis

    center_x = 1280 / 2
    center_y = 720 / 2

    current_x = 0.0  
    target_distance = 1.8 
    kp = 80   
    ki = 5    
    kd = 25   
    tolerance = 0.01   

    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    ep_gimbal.moveto(pitch=-10, yaw=0).wait_for_completed()

    overall_start_time = time.time()
    prev_error = 0.0
    integral = 0.0
    prev_time = overall_start_time

    background = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
    background = cv2.cvtColor(background, cv2.COLOR_BGR2GRAY)
    background = cv2.GaussianBlur(background, (21, 21), 0)

    while True:
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_frame = cv2.GaussianBlur(gray_frame, (21, 21), 0)

        diff_frame = cv2.absdiff(background, gray_frame)

        _, thresh_frame = cv2.threshold(diff_frame, 25, 255, cv2.THRESH_BINARY)

        thresh_frame = cv2.dilate(thresh_frame, None, iterations=2)

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_chick = np.array([33, 150, 100])
        upper_chick = np.array([36, 255, 255])

        lower_bottle = np.array([95, 80, 100])
        upper_bottle = np.array([120, 255, 255])

        mask_chick = cv2.inRange(hsv_frame, lower_chick, upper_chick)
        mask_bottle = cv2.inRange(hsv_frame, lower_bottle, upper_bottle)

        result_chick = cv2.bitwise_and(frame, frame, mask=mask_chick)
        result_bottle = cv2.bitwise_and(frame, frame, mask=mask_bottle)

        contours_chick, _ = cv2.findContours(
            mask_chick.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        contours_bottle, _ = cv2.findContours(
            mask_bottle.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if contours_chick or contours_bottle:
            chick_detect()
            bottle_detect()
            if target_distance - current_x > tolerance :
                current_time = time.time()
                error = target_distance - current_x
                time_diff = current_time - prev_time
                integral += error * time_diff
                derivative = (error - prev_error) / time_diff if time_diff > 0 else 0.0
                speed = kp * error + kd * derivative + ki * integral 
                speed = max(min(speed, 30), 0)  
                ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
                prev_error = error
                prev_time = current_time
                time.sleep(0.1)  

            else:
                ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
                time.sleep(1)
                continue


        cv2.imshow("Original Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        time.sleep(0.1)

    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_chassis.unsub_position()
    ep_robot.close()