# Arduino Face Tracker

A Python script that tracks head movements and controls Arduino servos via serial communication.

### Key Features
* **AI-Driven Hardware**: Uses Python (OpenCV and mediapipe) to detect head pose and drive Arduino servos instantly.
* **Low Latency**: Uses 115200 baud rate for fast response.
* **Stable**: Implements a "dead zone" logic to avoid servo jitter.
* **Lightweight**: Sends simple integer data instead of heavy strings.

### Usage
1.  Connect servos to Pin 6 (Y-axis) and Pin 9 (X-axis).
2.  Upload `arduino_code.ino`.
3.  Edit the port in `main.py` (e.g., `/dev/ttyUSB0` or `COM3`).
4.  Run the script.

### Requirements
* Python 3.x
* OpenCV, MediaPipe, PySerial
