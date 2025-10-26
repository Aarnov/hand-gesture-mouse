import cv2
import mediapipe as mp
import pyautogui
import math
import numpy as np

# --- Configuration Constants ---
# Smoothing factor to make mouse movement less jittery (higher value = smoother)
SMOOTHING = 7

# Define an "active area" on the camera feed to map to the screen.
# This creates a "dead zone" at the edges, making it easier to control.
# Values are percentages of the frame (0.0 to 1.0)
CAM_ACTIVE_X_START = 0.1  # 10% from the left
CAM_ACTIVE_X_END = 0.9    # 10% from the right
CAM_ACTIVE_Y_START = 0.1  # 10% from the top
CAM_ACTIVE_Y_END = 0.9    # 10% from the bottom

# Threshold for pinch-to-click (distance between thumb and index finger)
# You may need to adjust this value based on your camera and preference.
PINCH_THRESHOLD = 30

# --- Initialization ---

# Enable PyAutoGUI's failsafe (move mouse to a corner to stop)
pyautogui.FAILSAFE = True

# Get screen dimensions
SCREEN_WIDTH, SCREEN_HEIGHT = pyautogui.size()

# Initialize webcam
# --- MODIFIED LINE ---
# Added cv2.CAP_DSHOW. This is a common fix for Windows
# to prevent "green lines" and improve camera stability.
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
# --- END OF MODIFIED LINE ---

# --- NEW LINES TO SET A STABLE RESOLUTION ---
# We set a lower resolution (640x480) to improve performance and stability
# This often fixes issues with "choppy" video or green/corrupted frames.
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# --- END OF NEW LINES ---

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Get camera frame dimensions (we'll read one frame to get this)
success, frame = cap.read()
if not success:
    print("Error: Could not read frame from webcam.")
    cap.release()
    exit()
FRAME_HEIGHT, FRAME_WIDTH, _ = frame.shape

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)
mp_drawing = mp.solutions.drawing_utils

# --- State Variables ---
# To prevent rapid-fire clicking, we'll use a state variable
click_state = False
# Variables for smoothing mouse movement
prev_mouse_x, prev_mouse_y = pyautogui.position()

print("Hand tracker initialized. Move your hand in front of the camera.")
print(f"Screen size: {SCREEN_WIDTH}x{SCREEN_HEIGHT}")
print("Move mouse to any corner to quit.")

# --- Main Loop ---
try:
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        # --- MODIFIED LINE ---
        # Flip the frame horizontally for a natural mirror view
        # We are commenting this out, as it might conflict with the
        # cv2.CAP_DSHOW driver and cause the "inverting" bug.
        # frame = cv2.flip(frame, 1)
        # --- END OF MODIFIED LINE ---

        # Convert the BGR image to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process the frame to find hands
        results = hands.process(rgb_frame)

        # Draw a rectangle for the active area for visualization
        active_x1 = int(FRAME_WIDTH * CAM_ACTIVE_X_START)
        active_y1 = int(FRAME_HEIGHT * CAM_ACTIVE_Y_START)
        active_x2 = int(FRAME_WIDTH * CAM_ACTIVE_X_END)
        active_y2 = int(FRAME_HEIGHT * CAM_ACTIVE_Y_END)
        cv2.rectangle(frame, (active_x1, active_y1), (active_x2, active_y2), (0, 255, 0), 2)

        # If a hand is detected
        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]

            # Draw hand landmarks
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Get coordinates for thumb tip (ID 4) and index finger tip (ID 8)
            thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
            index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]

            # Get pixel coordinates from normalized coordinates
            # These are the coordinates on the *camera frame*
            index_tip_x = int(index_tip.x * FRAME_WIDTH)
            index_tip_y = int(index_tip.y * FRAME_HEIGHT)
            thumb_tip_x = int(thumb_tip.x * FRAME_WIDTH)
            thumb_tip_y = int(thumb_tip.y * FRAME_HEIGHT)

            # --- 1. Mouse Movement (Controlled by Index Finger) ---

            # Map the index finger's camera coordinates to the screen coordinates
            # We use np.interp for smooth mapping from the "active area"
            target_screen_x = np.interp(
                index_tip_x,
                (active_x1, active_x2),
                (0, SCREEN_WIDTH)
            )
            target_screen_y = np.interp(
                index_tip_y,
                (active_y1, active_y2),
                (0, SCREEN_HEIGHT)
            )

            # Smooth the movement
            current_mouse_x = prev_mouse_x + (target_screen_x - prev_mouse_x) / SMOOTHING
            current_mouse_y = prev_mouse_y + (target_screen_y - prev_mouse_y) / SMOOTHING

            # Move the mouse
            pyautogui.moveTo(current_mouse_x, current_mouse_y)

            # Update the previous position for the next frame's smoothing
            prev_mouse_x, prev_mouse_y = current_mouse_x, current_mouse_y

            # --- 2. Click Detection (Pinch Gesture) ---

            # Calculate the distance between thumb and index finger
            distance = math.hypot(thumb_tip_x - index_tip_x, thumb_tip_y - index_tip_y)

            # Draw a line between the two tips
            cv2.line(frame, (thumb_tip_x, thumb_tip_y), (index_tip_x, index_tip_y), (255, 0, 0), 3)

            # Check for pinch
            if distance < PINCH_THRESHOLD:
                # If we are not already in a "clicked" state, click once
                if not click_state:
                    pyautogui.click()
                    click_state = True
                    print("Click!")
                # Change line color to green when pinching
                cv2.line(frame, (thumb_tip_x, thumb_tip_y), (index_tip_x, index_tip_y), (0, 255, 0), 3)
            else:
                # Reset the click state if fingers are apart
                click_state = False

        # Display the frame
        cv2.imshow('Hand Controlled Mouse', frame)

        # Exit loop if 'q' is pressed
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

except pyautogui.FailSafeException:
    print("Failsafe triggered. Exiting.")
except KeyboardInterrupt:
    print("Program stopped by user.")
finally:
    # Release resources
    cap.release()
    cv2.destroyAllWindows()
    hands.close()
    print("Resources released. Goodbye!")

