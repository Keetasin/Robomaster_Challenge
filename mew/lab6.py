import cv2
import time
import numpy as np
from robomaster import robot, camera
import matplotlib.pyplot as plt

def capture_image(ep_camera, distance):
    # Start video stream
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    
    # Wait for the image to stabilize
    time.sleep(0.5)
    
    # Get the latest image
    img = ep_camera.read_cv2_image(strategy="newest")[250:600,300:980]
    
    # Save the image
    cv2.imwrite(f"coke_can_{distance}cm.png", img)
    
    # Stop video stream
    ep_camera.stop_video_stream()
    
    return img

def measure_dimensions(img):
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

        cv2.putText(img, f"Coke can x: {x}, y: {y}, w: {w}, h: {h}", (x, y - 10), 
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Show the image with detection (optional)
        cv2.imshow("Detected Coke Can", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        # Return the dimensions in pixels
        return w, h  # width, height in pixels

    print("No contours found.")
    return 0, 0

if __name__ == '__main__':
    # Start the robot
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal

    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()

    # Set distances
    distances = [60, 120, 180]
    
    # Store data
    pixel_dimensions = []
    physical_dimensions = (5.45, 14.6)  # Dimensions of Coca-Cola can (Width, Height) in cm

    for distance in distances:
        img = capture_image(ep_camera, distance)
        pixel_width, pixel_height = measure_dimensions(img)
        pixel_dimensions.append((pixel_width, pixel_height))

    ep_robot.close()

    # Calculate constants kx_f and ky_f
    kx_values = []
    ky_values = []

    for i, distance in enumerate(distances):
        pixel_width, pixel_height = pixel_dimensions[i]
        if pixel_width > 0 and pixel_height > 0:  # Avoid division by zero
            # Updated formulas with correct scaling
            kx_f = pixel_width * distance / physical_dimensions[0]
            ky_f = pixel_height * distance / physical_dimensions[1]
            kx_values.append(kx_f)
            ky_values.append(ky_f)

    # Calculate averages
    average_kx_f = sum(kx_values) / len(kx_values) if kx_values else 0
    average_ky_f = sum(ky_values) / len(ky_values) if ky_values else 0

    print(f"Average kx_f: {average_kx_f}")
    print(f"Average ky_f: {average_ky_f}")

    # Test the formulas with new distances
    test_distances = [75, 100, 125]
    expected_pixel_dimensions = []

    for distance in test_distances:
        # Corrected formula for expected pixel dimensions
        expected_width = average_kx_f / (distance / physical_dimensions[0])
        expected_height = average_ky_f / (distance / physical_dimensions[1])
        expected_pixel_dimensions.append((expected_width, expected_height))

    print("Expected pixel dimensions for test distances:")
    for distance, (width, height) in zip(test_distances, expected_pixel_dimensions):
        print(f"Distance: {distance} cm -> Width: {width}, Height: {height}")

    # Plotting the graph
    physical_distances = distances + test_distances
    pixel_widths = [dim[0] for dim in pixel_dimensions] + [dim[0] for dim in expected_pixel_dimensions]
    pixel_heights = [dim[1] for dim in pixel_dimensions] + [dim[1] for dim in expected_pixel_dimensions]

    plt.figure(figsize=(10, 5))
    plt.scatter(physical_distances, pixel_widths, color='blue', label='Width (px)')
    plt.scatter(physical_distances, pixel_heights, color='red', label='Height (px)')
    plt.xlabel('Physical Distance (cm)')
    plt.ylabel('Pixel Distance (px)')
    plt.title('Physical Distance vs Pixel Distance')
    plt.legend()
    plt.grid()
    plt.show()
