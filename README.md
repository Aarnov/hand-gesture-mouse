Hand-Controlled Virtual Mouse

Description

This project allows you to control your computer's mouse cursor using hand gestures captured through your webcam. It uses OpenCV for video capture and MediaPipe for real-time hand tracking.

The script is designed to be robust, with strict gesture recognition to prevent accidental inputs.

Features

Relative Mouse Movement: Control the cursor by pointing with your index finger. The "clutch" system allows you to reposition your hand without moving the cursor.

Left Click: Pinch your thumb and index finger together.

Double Click: Pinch twice quickly.

Right Click: Make a "thumbs up" gesture (thumb pointing vertically, all other fingers in a fist).

Scrolling: Hold an open palm (all five fingers extended) and move your hand up or down.

Requirements

You will need Python 3 and the following libraries. You can install them using pip:

pip install opencv-python mediapipe pyautogui pywin32


Usage

Ensure you have a webcam connected to your computer.

Run the script from your terminal:

python hand_mouse.py


A window will open showing your webcam feed. Your hand gestures will now control your mouse.

To quit the program, press the 'q' key while the OpenCV window is active.

As a safety feature, PyAutoGUI's failsafe is enabled. You can forcefully stop the script at any time by moving your mouse cursor to any corner of the screen.

Configuration

You can fine-tune the script's performance by adjusting the constants at the top of the hand_mouse.py file:

CURSOR_SENSITIVITY: (Default: 2.0) Higher values make the mouse move faster.

SCROLL_SENSITIVITY: (Default: 1.5) Higher values make scrolling faster.

PINCH_THRESHOLD: (Default: 30) Increase this if clicks are hard to register; decrease it if clicks happen too easily.

THUMB_PERPENDICULAR_THRESHOLD: (Default: 25) Adjusts the strictness of the "thumbs up" gesture.