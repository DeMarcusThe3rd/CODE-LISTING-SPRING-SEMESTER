import cv2
import numpy as np
import picamera
import RPi.GPIO as GPIO
import time
from movement import Movement, Pid
import threading
from shapeSymbol import Symbols

# Pin configurations
IN1 = 17
IN2 = 22
IN3 = 23
IN4 = 24
ENA = 12
ENB = 18

# Initialize vehicle movement and PID controller
car = Movement(IN1, IN2, IN3, IN4, ENA, ENB)
pid = Pid(0.2, 0.0, 0.6, 20, 50, 0)

# Paths and names for template matching
template_paths = ['./w3symbols/realright.jpg']
template_names = ['Right Arrow']

# Initialize symbols detection
sym = Symbols()
sym.initTemplate(template_paths, template_names)

# Lists to store contour data
contours = [0,0,0]
c = [0,0,0]
area = [0,0,0]
cx = [0,0,0]
cy = [0,0,0]
dx = 0
dy = 0
toggle = 0

# Initialize streaming of video feed  
class WebcamStream:
    '''Class to manage webcam stream.'''
    def __init__(self, stream_id=0):
        self.stream_id = stream_id
        self.camera = cv2.VideoCapture(self.stream_id)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.camera.isOpened():
            print("[Exiting]: Error accessing webcam stream.")
            exit(0)

        _, self.frame = self.camera.read()
        self.stopped = True
        self.t = threading.Thread(target=self.update, args=())
        self.t.daemon = True

    def start(self):
        self.stopped = False
        self.t.start()

    def update(self):
        while True:
            if self.stopped:
                break
            _, self.frame = self.camera.read()
        self.camera.release()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True

def move(stream):
    '''Function to handle vehicle movement.'''
    while True: 
        frame = stream.read()
        frame = frame[160:320,0:640]
        hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Color masks
        red_lower = np.array([136, 70, 100], np.uint8) 
        red_upper = np.array([180, 255, 255], np.uint8) 
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 

        yellow_lower = np.array([15, 75, 75], np.uint8) 
        yellow_upper = np.array([45, 255, 255], np.uint8) 
        yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)

        black_lower = np.array([0, 0, 0], np.uint8) 
        black_upper = np.array([180, 255, 75], np.uint8) 
        black_mask = cv2.inRange(hsvFrame, black_lower, black_upper) 
        
        green_lower = np.array([60, 120, 100], np.uint8) 
        green_upper = np.array([90, 255, 255], np.uint8) 
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 
        
        blue_lower = np.array([100, 80, 2], np.uint8) 
        blue_upper = np.array([130, 255, 255], np.uint8) 
        blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper) 

        # Morphological transformations
        kernel = np.ones((5, 5), "uint8") 
        red_mask = cv2.dilate(red_mask, kernel) 
        yellow_mask = cv2.dilate(yellow_mask, kernel) 
        black_mask = cv2.dilate(black_mask, kernel) 

        # Contour detection for selected colours 
        contours[0], _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours[1], _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours[2], _ = cv2.findContours(black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

        # Process contours
        for i, contour in enumerate(contours):
            if contour:        
                c[i] = max(contours[i], key=cv2.contourArea)
                area[i] = cv2.contourArea(c[i])
            else:
                continue
        # Calculate centroids only if contour is larger than 100 pixels 
        if area[0] > 100:
            M = cv2.moments(c[0])
            cx[0], cy[0] = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
            dx = cx[0] - frame.shape[1] // 2
            dy = cy[0] - frame.shape[2] // 2
            
            # Contouring depends on area size, reset to 0 to refresh the contour calculation process
            area[0] = 0
            
            # Toggle to flag if track is detected 
            toggle = 1

        elif area[1] > 100:
            M = cv2.moments(c[1])
            cx[1], cy[1] = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
            dx = cx[1] - frame.shape[1] // 2
            dy = cy[1] - frame.shape[2] // 2
            area[1] = 0
            toggle = 1

        elif area[2] > 100:
            M = cv2.moments(c[2])
            cx[2], cy[2] = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
            dx = cx[2] - frame.shape[1] // 2
            dy = cy[2] - frame.shape[2] // 2
            area[2] = 0
            toggle = 1

        else:
            toggle = 0

        # Edge cases are prioritised 
        if toggle == 1:
            if sym.toggle == 1:
                car.stop(2, 0)
                car.forward(0.5, 25)
            elif dy > 120:
                if dx >= 0:
                    car.turn(0, 40, -70)
                elif dx < 0:
                    car.turn(0, -70, 40)
            elif dx < -200:
                car.turn(0,-50,50)
            elif dx > 200:
                car.turn(0,50,-50)
                
            # Default movement with PID
            else:
                pid.pid(dx)
                car.turn(0, pid.speedL, pid.speedR)
        elif toggle == 0:
            print("")

        # Display frame
        cv2.imshow("Color Detection", frame) 
        if cv2.waitKey(10) & 0xFF == ord('q'): 
            cv2.destroyAllWindows() 
            break

def symbols(stream):
    '''Function to handle symbol detection.'''
    while True:
        frame = stream.read()
        if sym.matchTemplate(frame) == 1:
            result = sym.TemplateResult
            print("Arrow Detected!")
        else:
            sym.shapeDetection(frame)
            result = sym.shapeResult

        cv2.imshow("frame", result)

        # Quit application when the 'q' key is pressed
        key = cv2.waitKey(1)
        if key == ord('q'):
            cv2.destroyAllWindows()
            GPIO.cleanup()

if __name__ == "__main__":
    # Initialize webcam stream thread and other threads 
    main_camera_stream = WebcamStream(stream_id=0)
    move_thread = threading.Thread(target=move, args=(main_camera_stream,))
    sym_thread = threading.Thread(target=symbols, args=(main_camera_stream,))

    # Start threads
    main_camera_stream.start()
    move_thread.start()
    sym_thread.start()
