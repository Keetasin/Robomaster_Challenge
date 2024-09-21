import cv2
import numpy as np
from robomaster import robot, camera

def detect_coke_can(ep_camera):
    # Start video stream
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    
    while True:
        # Get the latest image
        img = ep_camera.read_cv2_image(strategy="newest")[250:600, 300:980]

        # Convert the image to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define the red color ranges in HSV
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for the red color ranges
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # Combine the masks
        combined_mask = cv2.bitwise_or(mask1, mask2)

        # Find contours on the combined mask
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the largest contour
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Draw rectangle around the largest contour
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Annotate the image
            cv2.putText(img, f"Coke can x: {x}, y: {y}, w: {w}, h: {h}", 
                        (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Show the image with detection
        cv2.imshow("Detected Coke Can", img)

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Stop video stream
    ep_camera.stop_video_stream()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # Start the robot
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera

    # Start detecting Coke cans in real-time
    detect_coke_can(ep_camera)

    # Close the robot connection
    ep_robot.close()

