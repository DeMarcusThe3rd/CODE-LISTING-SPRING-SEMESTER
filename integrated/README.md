---

### Integrated Control Script

The integrated control script combines functionalities from multiple classes to control a Raspberry Pi vehicle. It utilizes multithreading to handle vehicle movement and symbol detection simultaneously.

#### Classes Used:

1. **Movement**: Manages vehicle movement by controlling motor pins and implementing basic movement operations such as forward, backward, stop, and turn.

2. **Pid**: Provides a PID controller for controlling the trajectory of the vehicle based on the detected symbols or contours.

3. **WebcamStream**: Handles the webcam stream for capturing live video feed used for color detection and symbol recognition.

4. **Symbols**: Performs template matching and basic shape detection to recognize predefined symbols (e.g., arrows, shapes) in the video feed.

#### Threads:

1. **Main Camera Stream Thread**: Manages the webcam stream for real-time video input. It continuously captures frames from the webcam and updates the frame data.

2. **Movement Thread**: Controls the movement of the vehicle based on color detection and PID-controlled trajectory adjustments. It processes color masks, detects contours, and calculates centroid deviations for determining movement commands.

3. **Symbols Thread**: Handles symbol detection using template matching and basic shape detection techniques. It matches template images with the input frames and detects predefined symbols or shapes. If a symbol is detected, it triggers specific actions such as adjusting the vehicle's movement.

#### Execution:

- The script initializes the webcam stream and starts the movement and symbol detection threads.
- The movement thread continuously processes color detection and PID-controlled trajectory adjustments based on the webcam feed.
- The symbols thread performs symbol detection using template matching and shape detection algorithms.
- The script terminates when the user presses the 'q' key, closing all windows and cleaning up GPIO resources.

---
