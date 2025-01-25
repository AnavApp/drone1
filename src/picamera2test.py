from picamera2 import Picamera2
import cv2
import numpy as np

# Initialize Picamera2
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"format": "BGR888", "size": (640, 480)}))
picam2.start()

# Function to calculate black and gray pixels
def count_black_and_gray_pixels(frame, gray_tolerance=10, black_threshold=30):
    # Calculate black pixels
    black_mask = cv2.inRange(frame, (0, 0, 0), (black_threshold, black_threshold, black_threshold))
    num_black_pixels = cv2.countNonZero(black_mask)

    # Calculate gray pixels
    gray_mask = cv2.inRange(
        cv2.absdiff(frame[:, :, 0], frame[:, :, 1]), 0, gray_tolerance
    ) & cv2.inRange(
        cv2.absdiff(frame[:, :, 1], frame[:, :, 2]), 0, gray_tolerance
    ) & cv2.inRange(
        cv2.absdiff(frame[:, :, 0], frame[:, :, 2]), 0, gray_tolerance
    )
    num_gray_pixels = cv2.countNonZero(gray_mask)

    return num_black_pixels, num_gray_pixels

# Open a window to display the live feed
cv2.namedWindow("Live Video")

while True:
    # Capture a frame
    frame = picam2.capture_array()

    # Count black and gray pixels
    num_black, num_gray = count_black_and_gray_pixels(frame)

    # Display the counts on the frame
    text = f"Black Pixels: {num_black}, Gray Pixels: {num_gray}"
    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Show the frame
    cv2.imshow("Live Video", frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cv2.destroyAllWindows()
picam2.stop()

# from picamera2 import Picamera2
# import cv2
# import numpy as np

# # Initialize Picamera2
# picam2 = Picamera2()
# picam2.configure(picam2.create_preview_configuration(main={"format": "BGR888", "size": (640, 480)}))
# picam2.start()

# while True:
#     # Capture a frame
#     frame = picam2.capture_array()

#     # Process the frame with OpenCV (e.g., display it or apply filters)
#     cv2.imshow("Camera Feed", frame)

#     # Break loop on 'q' key press
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # Cleanup
# cv2.destroyAllWindows()
# picam2.stop()
