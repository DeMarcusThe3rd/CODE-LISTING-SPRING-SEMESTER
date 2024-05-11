---

### Movement Class

The `Movement` class provides functionalities to control the movement of a Raspberry Pi vehicle using GPIO pins. It initializes GPIO pins for motor control and defines methods for performing various actions such as moving forward, backward, turning, and stopping.

#### Initialization

- GPIO pins for motor control (IN1, IN2, IN3, IN4, ENA, ENB) are initialized.
- PWM (pulse-width modulation) is set up for controlling motor speed.

#### Methods

1. `forward(t, PWM)`: Moves the vehicle forward for a specified duration (`t` milliseconds) with a given PWM value (`PWM`).
2. `backward(t, PWM)`: Moves the vehicle backward for a specified duration (`t` milliseconds) with a given PWM value (`PWM`).
3. `stop(t, PWM)`: Stops the vehicle for a specified duration (`t` milliseconds) with a given PWM value (`PWM`).
4. `turn(t, PWML, PWMR)`: Turns the vehicle for a specified duration (`t` milliseconds) with different PWM values for left and right motors (`PWML` and `PWMR`).

### Symbols Class

The `Symbols` class provides functionalities for symbol detection using OpenCV. It initializes template images for matching and defines methods for template matching and basic shape detection.

#### Initialization

- Template images for symbols (e.g., arrows, shapes) are loaded.
- The method for template matching (`cv2.TM_CCOEFF_NORMED`) is chosen.

#### Methods

1. `initTemplate(template_paths, template_names)`: Initializes template images for symbol detection.
2. `matchTemplate(tempimg)`: Matches template images with input images to detect symbols. If a match is found, it returns the image with detected symbols.
3. `symbolDetection(symimg)`: Detects basic shapes (e.g., circles, triangles, rectangles) in input images using contour detection and approximation. It returns the image with detected shapes.

---
