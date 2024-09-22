import cv2
import numpy as np
from robomaster import robot, camera


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="rndis")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal

    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()

    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    
    while True:
        img = ep_camera.read_cv2_image(strategy="newest")[250:600, 300:980]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        combined_mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)

            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cv2.putText(img, f"Coke can x: {x}, y: {y}, w: {w}, h: {h}", 
                        (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow("Detected Coke Can", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    ep_camera.stop_video_stream()
    cv2.destroyAllWindows()

    ep_robot.close()



# Average kx_f: 585.4545454545455
# Average ky_f: 562.5
# Expected pixel dimensions for test distances:
# Distance: 75 cm -> Width: 42.93333333333334, Height: 108.0
# Distance: 100 cm -> Width: 32.2, Height: 81.0
# Distance: 125 cm -> Width: 25.76, Height: 64.8