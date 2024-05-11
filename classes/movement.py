import RPi.GPIO as GPIO
import time

'''
 *   Hex to Binary Cheat Sheet
 *  ==========================================
 *   A = 1010 -> both motors forward
 *   9 = 1001 -> right forward, left backward
 *   6 = 0110 -> left forward, right backward
 *   5 = 0101 -> both motors backward
'''

class Movement:    
    def __init__(self, in1, in2, in3, in4, ena, enb):
        self.in1 = in1
        self.in2 = in2
        self.in3 = in3
        self.in4 = in4
        self.ena = ena
        self.enb = enb
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.in3, GPIO.OUT)
        GPIO.setup(self.in4, GPIO.OUT)

        GPIO.setup(self.ena, GPIO.OUT)
        GPIO.setup(self.enb, GPIO.OUT)
        self.leftpwm = GPIO.PWM(self.ena,75)
        self.rightpwm = GPIO.PWM(self.enb,75)
        self.leftpwm.start(0)
        self.rightpwm.start(0)
    
    # input as a user defined literal hex operator and converts to 4 bit binary, each bit for each of the 4 motor driver pins  */
    def send_to_driver(self, bit, PWML, PWMR):
        GPIO.output(self.in1, ((bit >> 3) & 0x1)) 
        GPIO.output(self.in2, ((bit >> 2) & 0x1)) 
        GPIO.output(self.in3, ((bit >> 1) & 0x1)) 
        GPIO.output(self.in4, ((bit >> 0) & 0x1)) 

        self.leftpwm.ChangeDutyCycle(PWML)
        self.rightpwm.ChangeDutyCycle(PWMR)

    #forward for time t in milliseconds */
    def forward(self, t, PWM):
        self.send_to_driver(0xA, PWM, PWM)
        time.sleep(t)

    #backward for time t in milliseconds */
    def backward(self, t, PWM):
        self.send_to_driver(0x5, PWM, PWM)
        time.sleep(t)

    #stop for time t in milliseconds */
    def stop(self, t, PWM):
        self.send_to_driver(0x0, PWM, PWM)
        time.sleep(t)

    #turns the vehicle for time t in milliseconds */
    def turn(self, t, PWML, PWMR):
        if(PWML < 0):   #left motor backward, right motor forward
            self.send_to_driver(0x9, -PWML, PWMR)
            time.sleep(t)
        
        elif(PWMR < 0):  #right motor backward, left motor forward
            self.send_to_driver(0x6, PWML, -PWMR)
            time.sleep(t)
        
        elif (PWML < 0) and (PWMR < 0):  #both motor backward
            self.send_to_driver(0x5, -PWML, -PWMR)
            time.sleep(t)
        
        else:   #both motor forward
            self.send_to_driver(0xA, PWML, PWMR)
            time.sleep(t)
               
class Pid:
    '''PID Class for controlling the vehicle's trajectory.'''
    def __init__(self, Kp, Ki, Kd, target, uplimit, dolimit):
        # PID constants
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        # Target value for the PID controller
        self.target = target
        
        # Upper and lower speed limits for motor control
        self.uplimit = uplimit
        self.dolimit = dolimit
        
        # Initialize PID components and error
        self.P = 0
        self.I = 0
        self.D = 0
        self.error = 0
        self.prev_error = 0
    
    def pid(self, dx: float):
        # Calculate current error
        self.error = dx
        
        # Calculate PID components
        self.P = self.error
        self.I += self.error
        self.D = self.error - self.prev_error
        
        # Update previous error
        self.prev_error = self.error
        
        # Calculate PID sum
        self.pidsum = ((self.Kp * self.P) + (self.Ki * self.I) + (self.Kd * self.D))
        
        # Calculate motor speeds based on PID sum and target value
        self.speedL = self.target - self.pidsum
        self.speedR = self.target + self.pidsum
        
        # Apply speed limits
        if self.speedL > self.uplimit:
            self.speedL = self.uplimit
        elif self.speedL < self.dolimit:
            self.speedL = self.dolimit
        
        if self.speedR > self.uplimit:
            self.speedR = self.uplimit
        elif self.speedR < self.dolimit:
            self.speedR = self.dolimit
