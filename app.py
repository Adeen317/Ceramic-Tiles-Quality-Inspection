import cv2
import serial
import time
import os
import numpy as np

# Set up serial communication with Arduino
ser = serial.Serial('COM4', 9600, timeout=1)  # Replace 'COM5' with your Arduino port
time.sleep(2)  # Wait for the serial connection to initialize

print("Serial initialized.")

IMAGES_PATH = os.path.join('data', 'images')
IMAGES_PATH1 = os.path.join('data', 'images_defected')

# Initialize the webcam
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Error: Could not open video capture")
    exit()

print("Webcam initialized.")

while True:
    ret, frame = cap.read()  # Capture frame-by-frame
    if not ret:
        print("Failed to grab frame")
        break

    frame1 = cv2.resize(frame, (0, 0), fx=0.95, fy=0.95)

    # Grayscale
    gray_frame = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)

    # Thresholding
    _, threshold = cv2.threshold(gray_frame, 88, 255, cv2.THRESH_BINARY)

    # Invert Image
    invert1 = cv2.bitwise_not(threshold)

    # Thresholding
    _, threshold3 = cv2.threshold(gray_frame, 56, 255, cv2.THRESH_BINARY)

    # Invert Image
    invert3 = cv2.bitwise_not(threshold3)

     # Define minimum area thresholds for cracks and spots
    min_area_threshold_crack1 = 50
    min_area_threshold_spot1 = 5

    min_area_threshold_crack2 = 50

    # Find contours in the binary image
    contours2, _ = cv2.findContours(invert3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Iterate through contours and draw bounding boxes
    for contour2 in contours2:
        area2 = cv2.contourArea(contour2)

        # Check if the contour area is above the threshold
        if area2 > min_area_threshold_crack2 and area2 < 20000:
            x2, y2, w2, h2 = cv2.boundingRect(contour2)
            aspect_ratio2 = float(w2) / h2
            cv2.rectangle(invert1, (x1, y1), (x1 + w1, y1 + h1), (255, 255, 255), 2)
            cv2.putText(invert1, 'Crack', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.rectangle(frame1, (x1, y1), (x1 + w1, y1 + h1), (255, 255, 255), 2)
            cv2.putText(frame1, 'Crack', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


    # Find contours in the binary image
    contours1, _ = cv2.findContours(invert1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Iterate through contours and draw bounding boxes for cracks and spots
    for contour1 in contours1:
        area1 = cv2.contourArea(contour1)
        x1, y1, w1, h1 = cv2.boundingRect(contour1)
        aspect_ratio1 = float(w1) / h1

        # Check if the contour area is above the threshold for cracks 
        if area1 > min_area_threshold_crack1 and area1 < 20000:
            #crack_detected = True
            cv2.rectangle(invert1, (x1, y1), (x1 + w1, y1 + h1), (255, 255, 255), 2)
            cv2.putText(invert1, 'Crack', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.rectangle(frame1, (x1, y1), (x1 + w1, y1 + h1), (255, 255, 255), 2)
            cv2.putText(frame1, 'Crack', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Check if the contour area is above the threshold for spots
        elif area1 > min_area_threshold_spot1 and area1 < 10:
            #spot_detected = True
            cv2.rectangle(frame1, (x1, y1), (x1 + w1, y1 + h1), (255, 255, 255), 2)
            cv2.putText(frame1, 'Spot', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.rectangle(invert1, (x1, y1), (x1 + w1, y1 + h1), (255, 255, 255), 2)
            cv2.putText(invert1, 'Spot', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


    # Convert grayscale invert1 to BGR
    invert1_bgr = cv2.cvtColor(invert1, cv2.COLOR_GRAY2BGR)

    # Horizontal stack
    hor = np.hstack((frame1, invert1_bgr))

    cv2.imshow('Webcam7', hor)

    if ser.in_waiting > 0:  # Check if there's a message from Arduino
        try:
            msg = ser.readline().decode('utf-8', errors='ignore').strip()  # Read the message
            print(f"Received message: {msg}")  # Debug print
        except UnicodeDecodeError as e:
            print(f"Error decoding message: {e}")
            continue  

        if "tile is detected" in msg:  # Check if the message contains the keyword
            #time.sleep(2)
            imgname = os.path.join(IMAGES_PATH, f"captured_image_{time.time()}.png")
            cv2.imwrite(imgname, frame)  # Save the captured image
            print(f"Image saved as {imgname}")
            ser.write(b'image captured\n')  # Send acknowledgment back to Arduino

            # Read the saved image
            image = cv2.imread(imgname)

            # Grayscale
            gray_frame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Thresholding
            _, threshold = cv2.threshold(gray_frame, 88, 255, cv2.THRESH_BINARY)

             # Thresholding
            _, threshold2 = cv2.threshold(gray_frame, 56, 255, cv2.THRESH_BINARY)

            # Invert Image
            invert = cv2.bitwise_not(threshold)
             # Invert Image
            invert2 = cv2.bitwise_not(threshold2)

            # Initialize variables to track if cracks and spots are detected
            crack_detected = False
            spot_detected = False

             # Find contours in the binary image
            contours1, _ = cv2.findContours(invert2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Iterate through contours and draw bounding boxes
            for contour1 in contours1:
                area1 = cv2.contourArea(contour1)

                # Check if the contour area is above the threshold
                if area1 > min_area_threshold_crack1 and area1 < 10000:
                    crack_detected = True
                    x1, y1, w1, h1 = cv2.boundingRect(contour1)
                    aspect_ratio1 = float(w1) / h1
                    cv2.rectangle(invert, (x1, y1), (x1 + w1, y1 + h1), (255, 255, 255), 2)
                    cv2.putText(invert, 'Crack', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Define minimum area thresholds for cracks and spots
            min_area_threshold_crack = 50
            min_area_threshold_spot = 5

            # Find contours in the binary image
            contours, _ = cv2.findContours(invert, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Iterate through contours and draw bounding boxes for cracks and spots
            for contour in contours:
                area = cv2.contourArea(contour)
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h

                # Check if the contour area is above the threshold for cracks
                if area > min_area_threshold_crack and area < 20000:
                    crack_detected = True
                    cv2.rectangle(invert, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    cv2.putText(invert, 'Crack', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # Check if the contour area is above the threshold for spots
                elif area > min_area_threshold_spot and area < 15:
                    spot_detected = True
                    cv2.rectangle(invert, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    cv2.putText(invert, 'Spot', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            if crack_detected or spot_detected:
                ser.write(b'tile is defected\n')  # Send defect signal to Arduino
                print("Defect is detected")

            cv2.imshow('Webcam3', invert)  # Display the resulting frame

            imgname1 = os.path.join(IMAGES_PATH1, f"captured_image_{time.time()}.png")
            cv2.imwrite(imgname1, invert)  # Save the captured image
            print(f"Image saved as {imgname1}")

    if cv2.waitKey(10) & 0xFF == ord('q'):  # Exit when 'q' is pressed
        break

cap.release()  # When everything done, release the capture
cv2.destroyAllWindows()  # Close all OpenCV windows
ser.close()  # Close the serial connection
print("Exited cleanly.")
