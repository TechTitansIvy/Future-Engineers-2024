import cv2
import numpy as np
import serial
import time

# Set up serial communication
arduino_port = '/dev/ttyUSB0'  # Adjust this based on your setup
baud_rate = 9600
arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)  # Give some time for the connection to establish

def find_centroid(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return None
    else:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)

cap = cv2.VideoCapture(0)

while True:
    read_ok, img = cap.read()
    if not read_ok:
        break

    img = cv2.resize(img, (640, 480))

    # Define ROI: lower half of the screen
    height, width, _ = img.shape
    roi = img[height//2:, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lower_red = np.array([170, 80, 0])
    upper_red = np.array([180, 255, 255])

    lower_green = np.array([60, 70, 0])
    upper_green = np.array([85, 255, 255])

    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([130, 255, 255])

    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    def is_inside(inner, outer):
        ix, iy, iw, ih = inner
        ox, oy, ow, oh = outer
        return ix >= ox and iy >= oy and ix + iw <= ox + ow and iy + ih <= oy + oh

    def merge_boxes(bounding_boxes):
        if not bounding_boxes:
            return []

        bounding_boxes.sort(key=lambda x: x[0])  # Sort by x coordinate
        merged_boxes = []

        current_box = bounding_boxes[0]
        for box in bounding_boxes[1:]:
            if is_inside(box, current_box):
                continue
            else:
                merged_boxes.append(current_box)
                current_box = box
        
        merged_boxes.append(current_box)  # Add the last box
        return merged_boxes

    def process_contours(contours, label, lower_color, upper_color):
        bounding_boxes = [cv2.boundingRect(cnt) for cnt in contours if cv2.contourArea(cnt) > 1000]
        bounding_boxes = merge_boxes(bounding_boxes)

        for box in bounding_boxes:
            x, y, w, h = box
            centroid = ((x + x + w) // 2, (y + y + h) // 2)
            cv2.circle(roi, centroid, 5, (0, 255, 0), -1)
            if label == 'Red':
                print("Turning right")
                state_input = "right"
                arduino.write(state_input.encode())
                arduino.write('\n'.encode())
                time.sleep(0.1)
            if label == 'Green':
                print("Turning left")
                state_input = "left"
                arduino.write(state_input.encode())
                arduino.write('\n'.encode())
                time.sleep(0.1)
            box_color = (0, 255, 0) 
            cv2.rectangle(roi, (x, y), (x + w, y + h), box_color, 2)
            cv2.putText(roi, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, box_color, 2)

        return len(bounding_boxes) > 0

    red_detected = process_contours(contours_red, 'Red', lower_red, upper_red)
    green_detected = process_contours(contours_green, 'Green', lower_green, upper_green)
    blue_detected = process_contours(contours_blue, 'Blue', lower_blue, upper_blue)

    if not red_detected and not green_detected and not blue_detected:
        print("Moving straight")
        state_input = "straight"
        arduino.write(state_input.encode())
        arduino.write('\n'.encode())
        time.sleep(0.1)

    cv2.imshow('Color Recognition Output', roi)

    if cv2.waitKey(1) & 0xFF == ord('x'):
        break

cap.release()
cv2.destroyAllWindows()
