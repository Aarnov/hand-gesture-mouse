import cv2
import mediapipe as mp
import pyautogui
import math
import numpy as np
import time
import win32gui
import win32con

# --- Configuration Constants ---
SMOOTHING = 3           # smaller = more responsive
CURSOR_SENSITIVITY = 7.0  # how much cursor moves relative to hand
SCROLL_SENSITIVITY = 1.5  # how much scroll moves relative to hand (Adjusted)
CAM_ACTIVE_X_START = 0.1
CAM_ACTIVE_X_END = 0.9
CAM_ACTIVE_Y_START = 0.1
CAM_ACTIVE_Y_END = 0.9
PINCH_THRESHOLD = 30      # Increased for easier clicking
DOUBLE_CLICK_WINDOW = 0.5
SCROLL_THRESHOLD = 5
SCROLL_SPEED = 50
DEADZONE = 10           # deadzone for scroll clutch
THUMB_PERPENDICULAR_THRESHOLD = 25 # Max horizontal distance for a "straight up" thumb

# --- Initialization ---
pyautogui.FAILSAFE = True
SCREEN_WIDTH, SCREEN_HEIGHT = pyautogui.size()

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

success, frame = cap.read()
if not success:
    print("Error: Could not read frame from webcam.")
    cap.release()
    exit()
FRAME_HEIGHT, FRAME_WIDTH, _ = frame.shape

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

click_state = False
right_click_state = False
last_click_time = 0
scroll_anchor = None      # Anchor point for scroll
scroll_state = None
prev_mouse_x, prev_mouse_y = pyautogui.position()
window_set_topmost = False

# --- State for Relative Mouse Movement ---
prev_hand_x, prev_hand_y = 0, 0
pointing_clutch_engaged = False

print("Hand tracker initialized. Move mouse to any corner to quit.")

# --- Helper function for 2D landmark distance ---
def get_dist(p1, p2):
    """Calculates the 2D Euclidean distance between two MediaPipe landmarks."""
    return math.hypot(p1.x - p2.x, p1.y - p2.y)

try:
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            continue

        # --- FLIP FRAME for intuitive control ---
        frame = cv2.flip(frame, 1)

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb_frame)

        active_x1 = int(FRAME_WIDTH * CAM_ACTIVE_X_START)
        active_y1 = int(FRAME_HEIGHT * CAM_ACTIVE_Y_START)
        active_x2 = int(FRAME_WIDTH * CAM_ACTIVE_X_END)
        active_y2 = int(FRAME_HEIGHT * CAM_ACTIVE_Y_END)
        # Green box is still drawn for visual feedback, but not used for cursor mapping
        cv2.rectangle(frame, (active_x1, active_y1), (active_x2, active_y2), (0, 255, 0), 2)

        if results.multi_hand_landmarks:
            lm = results.multi_hand_landmarks[0].landmark
            mp_drawing.draw_landmarks(frame, results.multi_hand_landmarks[0], mp_hands.HAND_CONNECTIONS)

            # --- Finger states (Robust, orientation-independent) ---
            
            wrist_lm = lm[mp_hands.HandLandmark.WRIST]
            
            # Finger extended: Tip is further from wrist than PIP joint
            finger_extended = [
                get_dist(lm[mp_hands.HandLandmark.INDEX_FINGER_TIP], wrist_lm) > get_dist(lm[mp_hands.HandLandmark.INDEX_FINGER_PIP], wrist_lm),
                get_dist(lm[mp_hands.HandLandmark.MIDDLE_FINGER_TIP], wrist_lm) > get_dist(lm[mp_hands.HandLandmark.MIDDLE_FINGER_PIP], wrist_lm),
                get_dist(lm[mp_hands.HandLandmark.RING_FINGER_TIP], wrist_lm) > get_dist(lm[mp_hands.HandLandmark.RING_FINGER_PIP], wrist_lm),
                get_dist(lm[mp_hands.HandLandmark.PINKY_TIP], wrist_lm) > get_dist(lm[mp_hands.HandLandmark.PINKY_PIP], wrist_lm)
            ]
            
            # Finger curled: Tip is closer to wrist than PIP joint
            finger_curled = [
                get_dist(lm[mp_hands.HandLandmark.INDEX_FINGER_TIP], wrist_lm) < get_dist(lm[mp_hands.HandLandmark.INDEX_FINGER_PIP], wrist_lm),
                get_dist(lm[mp_hands.HandLandmark.MIDDLE_FINGER_TIP], wrist_lm) < get_dist(lm[mp_hands.HandLandmark.MIDDLE_FINGER_PIP], wrist_lm),
                get_dist(lm[mp_hands.HandLandmark.RING_FINGER_TIP], wrist_lm) < get_dist(lm[mp_hands.HandLandmark.RING_FINGER_PIP], wrist_lm),
                get_dist(lm[mp_hands.HandLandmark.PINKY_TIP], wrist_lm) < get_dist(lm[mp_hands.HandLandmark.PINKY_PIP], wrist_lm)
            ]

            # --- Original finger_pointing logic (for left-click only) ---
            index_finger_pointing = lm[mp_hands.HandLandmark.INDEX_FINGER_TIP].y < lm[mp_hands.HandLandmark.INDEX_FINGER_PIP].y
            
            # thumb_curled: Tip is closer to index MCP than thumb IP joint is
            index_mcp_lm = lm[mp_hands.HandLandmark.INDEX_FINGER_MCP]
            thumb_tip_lm = lm[mp_hands.HandLandmark.THUMB_TIP]
            thumb_ip_lm = lm[mp_hands.HandLandmark.THUMB_IP]
            thumb_curled = get_dist(thumb_tip_lm, index_mcp_lm) < get_dist(thumb_ip_lm, index_mcp_lm)

            # Key points
            index_tip_x = int(lm[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * FRAME_WIDTH)
            index_tip_y = int(lm[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * FRAME_HEIGHT)
            thumb_tip_x = int(lm[mp_hands.HandLandmark.THUMB_TIP].x * FRAME_WIDTH)
            thumb_tip_y = int(lm[mp_hands.HandLandmark.THUMB_TIP].y * FRAME_HEIGHT)
            wrist_y = int(lm[mp_hands.HandLandmark.WRIST].y * FRAME_HEIGHT)
            wrist_x = int(lm[mp_hands.HandLandmark.WRIST].x * FRAME_WIDTH)

            # --- Distances ---
            dist_thumb_index = math.hypot(thumb_tip_x - index_tip_x, thumb_tip_y - index_tip_y)
            thumb_index_dist = math.hypot(
                lm[mp_hands.HandLandmark.THUMB_TIP].x - lm[mp_hands.HandLandmark.INDEX_FINGER_MCP].x,
                lm[mp_hands.HandLandmark.THUMB_TIP].y - lm[mp_hands.HandLandmark.INDEX_FINGER_MCP].y
            ) * FRAME_WIDTH

            # --- Gesture Properties (for stricter logic) ---
            thumb_is_pointing_up = lm[mp_hands.HandLandmark.THUMB_TIP].y < lm[mp_hands.HandLandmark.THUMB_MCP].y
            thumb_horizontal_dist = abs(lm[mp_hands.HandLandmark.THUMB_TIP].x - lm[mp_hands.HandLandmark.THUMB_MCP].x) * FRAME_WIDTH
            is_perpendicular_thumbs_up = thumb_is_pointing_up and (thumb_horizontal_dist < THUMB_PERPENDICULAR_THRESHOLD)
            
            # Strict pointing: Index out, others curled, thumb curled
            is_strict_pointing = finger_extended[0] and finger_curled[1] and finger_curled[2] and finger_curled[3] and thumb_curled

            # --- Scroll logic (robust with deadzone + anchor) ---
            # Gesture: Open palm (all fingers extended)
            if all(finger_extended):
                if scroll_anchor is None:
                    scroll_anchor = wrist_y
                    scroll_state = None

                delta = (wrist_y - scroll_anchor) * SCROLL_SENSITIVITY

                if delta > SCROLL_THRESHOLD:
                    pyautogui.scroll(-SCROLL_SPEED)
                    scroll_state = 'down'
                    scroll_anchor += SCROLL_THRESHOLD / SCROLL_SENSITIVITY
                    print("Scrolling Down")
                elif delta < -SCROLL_THRESHOLD:
                    pyautogui.scroll(SCROLL_SPEED)
                    scroll_state = 'up'
                    scroll_anchor -= SCROLL_THRESHOLD / SCROLL_SENSITIVITY
                    print("Scrolling Up")
                elif abs(delta) < DEADZONE:
                    scroll_state = None  # reset to allow re-arming

                cv2.putText(frame, "SCROLL MODE", (active_x1+10, active_y1+50),
                            cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
                cv2.circle(frame, (wrist_x, wrist_y), 15, (0, 0, 255), -1)
                click_state = False
                right_click_state = False
                pointing_clutch_engaged = False # Disengage pointing clutch

            # --- Left click (pinch) ---
            # Gesture: Index finger tip and thumb tip close together
            elif not index_finger_pointing and dist_thumb_index < PINCH_THRESHOLD:
                if not click_state:
                    now = time.time()
                    if now - last_click_time < DOUBLE_CLICK_WINDOW:
                        # This is the *second* click. The first was already sent.
                        # Send a *single* click, and the OS will interpret the pair as a double-click.
                        pyautogui.click() # <-- CHANGED from pyautogui.doubleClick()
                        last_click_time = 0 # Reset to prevent triple-click
                        print("Double Click!")
                    else:
                        # This is the *first* click.
                        pyautogui.click()
                        last_click_time = now # Set to 'now' to listen for a second click
                        print("Click!")
                    click_state = True
                cv2.line(frame,(thumb_tip_x,thumb_tip_y),(index_tip_x,index_tip_y),(0,255,0),3)
                scroll_anchor = None
                right_click_state = False
                pointing_clutch_engaged = False # Disengage pointing clutch

            # --- Right click (thumbs up) ---
            # Gesture: Thumb is pointing STRAIGHT up, all other 4 fingers are curled
            elif is_perpendicular_thumbs_up and all(finger_curled) and thumb_index_dist > 40:
                if not right_click_state:
                    pyautogui.rightClick()
                    right_click_state = True
                    print("Right Click!")
                cv2.circle(frame,(thumb_tip_x,thumb_tip_y),10,(0,255,0),-1)
                cv2.putText(frame, "RIGHT CLICK", (active_x1+10, active_y1+50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)
                scroll_anchor = None
                click_state = False
                pointing_clutch_engaged = False # Disengage pointing clutch

            # --- Mouse move (STRICT pointing) ---
            # Gesture: Index finger extended, all other 3 fingers + thumb curled
            elif is_strict_pointing:
                scroll_anchor = None
                click_state = False
                right_click_state = False
                cv2.putText(frame, "MOVING CURSOR", (active_x1+10, active_y1+50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255),2)

                # --- Relative (Clutch) Movement Logic ---
                if not pointing_clutch_engaged:
                    pointing_clutch_engaged = True
                    # Set the 'previous' hand position to the current one on first frame
                    prev_hand_x, prev_hand_y = index_tip_x, index_tip_y
                    # Get the most recent mouse position as the starting point
                    prev_mouse_x, prev_mouse_y = pyautogui.position()
                
                # Set target_x/y to be relative to the hand's delta
                # This works with the existing smoothing logic below
                target_x = prev_mouse_x + (index_tip_x - prev_hand_x)
                target_y = prev_mouse_y + (index_tip_y - prev_hand_y)
                
                # Apply smoothing
                dx = (target_x - prev_mouse_x) * CURSOR_SENSITIVITY
                dy = (target_y - prev_mouse_y) * CURSOR_SENSITIVITY

                current_mouse_x = prev_mouse_x + dx / SMOOTHING
                current_mouse_y = prev_mouse_y + dy / SMOOTHING
                
                pyautogui.moveTo(current_mouse_x, current_mouse_y)
                
                # Update prev_mouse for the *next* frame's smoothing calc
                prev_mouse_x, prev_mouse_y = current_mouse_x, current_mouse_y
                # Update prev_hand for the *next* frame's delta calc
                prev_hand_x, prev_hand_y = index_tip_x, index_tip_y

                cv2.circle(frame,(index_tip_x,index_tip_y),10,(0,255,255),-1)

            # --- Idle ---
            else:
                scroll_anchor = None
                scroll_state = None
                click_state = False
                right_click_state = False
                pointing_clutch_engaged = False # Disengage pointing clutch

        else:
            # No hand detected
            scroll_anchor = None
            scroll_state = None
            click_state = False
            right_click_state = False
            pointing_clutch_engaged = False # Disengage pointing clutch

        cv2.imshow('Hand Controlled Mouse', frame)

        # Always on top
        if not window_set_topmost:
            try:
                hwnd = win332gui.FindWindow(None,'Hand Controlled Mouse')
                if hwnd:
                    win32gui.SetWindowPos(hwnd,win32con.HWND_TOPMOST,0,0,0,0,
                                        win32con.SWP_NOMOVE|win32con.SWP_NOSIZE)
                    window_set_topmost = True
            except: 
                window_set_topmost = True # Give up if it fails

        if cv2.waitKey(5)&0xFF==ord('q'):
            break

except pyautogui.FailSafeException:
    print("Failsafe triggered.")
except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    cap.release()
    cv2.destroyAllWindows()
    hands.close()
    print("Resources released.")

