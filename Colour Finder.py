import cv2
import numpy as np

def on_trackbar_change(value):
    pass

def color_detection_camera():
    # Initial HSV values
    min_hue, min_saturation, min_value = 30, 50, 50
    max_hue, max_saturation, max_value = 60, 255, 255

    # Create a resizable window
    cv2.namedWindow('Color Detection', cv2.WINDOW_NORMAL)

    # Create trackbars for adjusting HSV values
    cv2.createTrackbar('Min Hue', 'Color Detection', min_hue, 180, on_trackbar_change)
    cv2.createTrackbar('Min Saturation', 'Color Detection', min_saturation, 255, on_trackbar_change)
    cv2.createTrackbar('Min Value', 'Color Detection', min_value, 255, on_trackbar_change)
    cv2.createTrackbar('Max Hue', 'Color Detection', max_hue, 180, on_trackbar_change)
    cv2.createTrackbar('Max Saturation', 'Color Detection', max_saturation, 255, on_trackbar_change)
    cv2.createTrackbar('Max Value', 'Color Detection', max_value, 255, on_trackbar_change)

    # Open the default camera (usually camera index 0)
    cap = cv2.VideoCapture(0)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Resize the frame to a smaller size (640x480)
        resized_frame = cv2.resize(frame, (640, 480))

        # Convert the frame from BGR to HSV
        hsv_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2HSV)

        # Get current trackbar positions
        min_hue = cv2.getTrackbarPos('Min Hue', 'Color Detection')
        min_saturation = cv2.getTrackbarPos('Min Saturation', 'Color Detection')
        min_value = cv2.getTrackbarPos('Min Value', 'Color Detection')
        max_hue = cv2.getTrackbarPos('Max Hue', 'Color Detection')
        max_saturation = cv2.getTrackbarPos('Max Saturation', 'Color Detection')
        max_value = cv2.getTrackbarPos('Max Value', 'Color Detection')

        # Define the HSV color range for the color you want to detect
        lower_color = np.array([min_hue, min_saturation, min_value])
        upper_color = np.array([max_hue, max_saturation, max_value])

        # Create a mask using the specified color range
        color_mask = cv2.inRange(hsv_frame, lower_color, upper_color)

        # Apply the mask to the resized frame to extract the colored regions
        result_frame = cv2.bitwise_and(resized_frame, resized_frame, mask=color_mask)

        # Display the original resized frame and the result
        cv2.imshow('Color Detection', np.hstack([resized_frame, result_frame]))

        # Break the loop when the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()

# Call the color_detection_camera function
color_detection_camera()
