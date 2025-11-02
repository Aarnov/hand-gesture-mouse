# Hand-Controlled Virtual Mouse

## Description

This project allows you to control your computer's mouse cursor using **hand gestures** captured through your webcam.  
It uses **OpenCV** for video capture and **MediaPipe** for real-time hand tracking.

The script is designed to be robust, with strict gesture recognition to prevent accidental inputs.

---

## Features

### 1. Mouse Movement
- **Gesture:** Point with your **index finger**  
- **Action:** Controls cursor movement  
- **Details:** A built-in *clutch system* allows you to reposition your hand without moving the cursor

### 2. Left Click
- **Gesture:** Pinch your **thumb and index finger** together  
- **Action:** Performs a left mouse click

### 3. Double Click
- **Gesture:** Pinch twice quickly  
- **Action:** Performs a double click

### 4. Right Click
- **Gesture:** Make a **thumbs-up gesture** (thumb vertical, other fingers closed)  
- **Action:** Performs a right mouse click

### 5. Scrolling
- **Gesture:** Open palm (all five fingers extended) and move your hand **up or down**  
- **Action:** Scrolls vertically on the screen

---

## Requirements

You will need **Python 3** and the following libraries:

```bash
pip install opencv-python mediapipe pyautogui pywin32
