import cv2
import numpy as np
from robomaster import robot, camera


if __name__ == '_main_':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal

    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()

    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    
    while True:
        img = ep_camera.read_cv2_image(strategy="newest")[:600,:]

        # Draw crosshair at the center of the image
        height, width = img.shape[:2]
        center_x, center_y = width // 2, height // 2
        
        # Draw horizontal and vertical lines (crosshair)
        cv2.line(img, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 2)  # Horizontal line
        cv2.line(img, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 2)  # Vertical line
        



        cv2.imshow("Detected Coke Can", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    ep_camera.stop_video_stream()
    cv2.destroyAllWindows()

    ep_robot.close()