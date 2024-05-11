---

# Computational Vision Line Following Vehicle

## Overview

This project presents the development and implementation of a computational vision-based line-following robot utilizing the Raspberry Pi 4 as the processor. The vehicle demonstrates the ability to autonomously track coloured lines with HSV colour masking and centroid deviations, perform symbol and shape detection through template matching and polygon approximation techniques, respectively. The key features of the system include the integration and simultaneous functionality of all capabilities with implementations such as a PID controller to ensure smooth trajectory tracking and the utilization of rotary encoders for distance tracking and angle measurement, being secondary functionalities to the core computational vision tasks.

## Components

### 1. Movement Control
- **Description**: Implements motor control for vehicle movement including forward, backward, stop, and turning operations.
- **Files**: `movement.py`
- **Usage**: Interfaces with GPIO pins to control motor drivers and provides methods for basic vehicle movement.

### 2. PID Controller
- **Description**: Implements a Proportional-Integral-Derivative (PID) controller for trajectory adjustments based on visual input.
- **Files**: `movement.py`
- **Usage**: Calculates motor speeds based on centroid deviations for precise vehicle navigation.

### 3. Webcam Stream
- **Description**: Manages live video streaming from a webcam for color detection and symbol recognition.
- **Files**: Included in `trackChallenge.py`
- **Usage**: Captures video frames and provides methods for accessing and processing the live feed.

### 4. Symbol Recognition
- **Description**: Performs template matching and basic shape detection to recognize predefined symbols (e.g., arrows, shapes) in the video feed.
- **Files**: `shapesymbol.py`
- **Usage**: Matches template images with input frames and triggers specific actions based on detected symbols.

### 5. Integrated Control Script
- **Description**: Combines functionalities from the above components to control the Raspberry Pi vehicle.
- **Files**: `integrated/trackChallenge.py`
- **Usage**: Utilizes multithreading to manage vehicle movement, symbol detection, and trajectory adjustments simultaneously.

## Installation

1. Clone the repository to your Raspberry Pi.
2. Ensure all required dependencies are installed (OpenCV, NumPy, etc.).
3. Connect the motor drivers and webcam to appropriate GPIO pins.
4. Customize configurations and parameters in the control script if necessary.

## Usage

1. Run the integrated control script (`trackChallenge.py`) to start the vehicle control system.
2. Follow on-screen instructions or configure remote control options as needed.
3. Terminate the script by pressing the 'q' key to exit the application and release GPIO resources.

## Contributing

Contributions to improve functionality, add features, or fix bugs are welcome! Please open an issue to discuss proposed changes before submitting a pull request.

## License

This project is licensed under the [MIT License](LICENSE).

---

