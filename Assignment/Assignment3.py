# import cv2
# import robomaster
# from robomaster import robot, camera
# import time
# import numpy as np


# class Marker:
#     def __init__(self, x, y, w, h, score):
#         self._x = x
#         self._y = y
#         self._w = w
#         self._h = h
#         self._score = score

#     @property
#     def pt1(self):
#         return int(self._x), int(self._y)

#     @property
#     def pt2(self):
#         return int(self._x + self._w), int(self._y + self._h)

#     @property
#     def center(self):
#         return int(self._x + self._w / 2), int(self._y + self._h / 2)

#     @property
#     def text(self):
#         return f"Score: {self._score:.2f}"


# def sub_position_handler(position_info):
#     global current_x
#     current_x, y, z = position_info


# def detect_chicken():
#     max_chicken_contour = max(contours_chicken, key=cv2.contourArea)
#     (x_chicken, y_chicken, w_chicken, h_chicken) = cv2.boundingRect(max_chicken_contour)
    
#     if w_chicken > 65:
#         add_w_chicken = int(w_chicken * 0.3)
#         add_h_chicken = int(h_chicken * 0.6)
#         new_y_chicken = y_chicken - add_h_chicken // 2 + 20

#     elif w_chicken > 25:
#         add_w_chicken = int(w_chicken * 0.35)
#         add_h_chicken = int(h_chicken * 0.55)
#         new_y_chicken = (y_chicken - add_h_chicken // 2 + 20) - 15

#     else:
#         add_w_chicken = int(w_chicken * 0.39)
#         add_h_chicken = int(h_chicken * 0.59)
#         new_y_chicken = (y_chicken - add_h_chicken // 2 + 20) - 15

#     new_x_chicken = x_chicken - add_w_chicken // 2
#     new_w_chicken = w_chicken + add_w_chicken
#     new_h_chicken = h_chicken + add_h_chicken

#     # cv2.rectangle(frame, (new_x_chicken, new_y_chicken), (new_x_chicken + new_w_chicken, new_y_chicken + new_h_chicken), (255, 0, 255), 2)
#     # cv2.putText(frame, f" chicken x: {x_chicken}, y: {y_chicken}, w: {w_chicken}, h: {h_chicken}", (x_chicken, y_chicken - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
#     return (new_x_chicken, new_y_chicken)

# if __name__ == "__main__":
#     ep_robot = robot.Robot()
#     ep_robot.initialize(conn_type="ap")
#     ep_camera = ep_robot.camera
#     ep_gimbal = ep_robot.gimbal
#     ep_chassis = ep_robot.chassis
#     ep_blaster = ep_robot.blaster

#     center_x = 1280 / 2
#     center_y = 720 / 2

#     current_x = 0.0  
#     target_distance = 1.3
#     kp, ki, kd = 75, 10, 30   
#     tolerance = 0.01   
#     prev_error, integral = 0.0, 0.0
#     prev_time = time.time()

#     p = 0.6 / 1.7
#     i = p / (0.7 / 2)
#     d = p * (0.7 / 8)
#     boom = True

#     accumulate_err_x = 0
#     accumulate_err_y = 0
#     data_pith_yaw = []
#     prev_time = time.time()
#     prev_err_x = 0
#     prev_err_y = 0


#     ep_chassis.sub_position(freq=10, callback=sub_position_handler)
#     ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
#     ep_gimbal.sub_angle(freq=10)
#     ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
#     # ep_gimbal.moveto(pitch=-10, yaw=0).wait_for_completed()

#     while True:
#         frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)[250:600,300:980]

#         hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

#         lower_chicken = np.array([33, 150, 100])
#         upper_chicken = np.array([38, 255, 255])

#         mask_chicken = cv2.inRange(hsv_frame, lower_chicken, upper_chicken)

        
#         contours_chicken, _ = cv2.findContours(mask_chicken, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         if contours_chicken: marker = detect_chicken()

#         # if target_distance - current_x > tolerance :
#         #     current_time = time.time()
#         #     error = target_distance - current_x
#         #     time_diff = current_time - prev_time
#         #     integral += error * time_diff
#         #     derivative = (error - prev_error) / time_diff if time_diff > 0 else 0.0
#         #     speed = kp * error + kd * derivative + ki * integral 
#         #     speed = max(min(speed, 20), 0)  
#         #     ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
#         #     prev_error = error
#         #     prev_time = current_time
#         #     time.sleep(0.05)  

#         # else:
#         #     ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
#         #     time.sleep(0.05)
        
#         if marker:
#             after_time = time.time()
#             x, y = marker.center

#             err_x = center_x - x
#             err_y = center_y - y
#             accumulate_err_x += err_x * (after_time - prev_time)
#             accumulate_err_y += err_y * (after_time - prev_time)

    
#             speed_x = p * err_x
#             speed_y = p * err_y
#             ep_gimbal.drive_speed(pitch_speed=speed_y, yaw_speed=-speed_x)

#             data_pith_yaw.append([err_x, err_y, round(speed_x, 3), round(speed_y, 3), after_time - prev_time])

#             prev_time = after_time
#             prev_err_x = err_x
#             prev_err_y = err_y

#             cv2.rectangle(frame, marker.pt1, marker.pt2, (0, 255, 0), 2)
#             cv2.putText(frame, marker.text, marker.center, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
#             time.sleep(0.001)

#         else:
#             ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        
#         cv2.imshow("Frame", frame)
#         if cv2.waitKey(1) & 0xFF == ord("q"): break

#         time.sleep(0.1)

#     cv2.destroyAllWindows()
#     ep_camera.stop_video_stream()
#     ep_chassis.unsub_position()
#     ep_robot.close()







import cv2
import robomaster
from robomaster import robot, camera
import time
import numpy as np

class Marker:
    def __init__(self, x, y, w, h, score):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._score = score

    @property
    def pt1(self):
        return int(self._x), int(self._y)

    @property
    def pt2(self):
        return int(self._x + self._w), int(self._y + self._h)

    @property
    def center(self):
        return int(self._x + self._w / 2), int(self._y + self._h / 2)

    @property
    def text(self):
        return f"Score: {self._score:.2f}"

def sub_position_handler(position_info):
    global current_x
    current_x, y, z = position_info

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

    return Marker(new_x_chicken, new_y_chicken, new_w_chicken, new_h_chicken, score=1.0)

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

    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_gimbal.sub_angle(freq=10)
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()

    while True:
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)[250:600,300:980]
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_chicken = np.array([33, 150, 100])
        upper_chicken = np.array([38, 255, 255])

        mask_chicken = cv2.inRange(hsv_frame, lower_chicken, upper_chicken)
        contours_chicken, _ = cv2.findContours(mask_chicken, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        marker = None
        if contours_chicken:
            marker = detect_chicken(contours_chicken)

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

            cv2.rectangle(frame, marker.pt1, marker.pt2, (0, 255, 0), 2)
            cv2.putText(frame, marker.text, marker.center, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
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
