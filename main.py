import time
import board
import RPi.GPIO as GPIO
from adafruit_motorkit import MotorKit

# Set GPIO numbering mode, motor kit is setting the mode as BCM by default so below is not needed
GPIO.setmode(GPIO.BCM)
# Set pin 11 as an output, and set servo1 as pin 11 as PWM
GPIO.setup(11, GPIO.OUT)
THRESHOLD_DISTANCE = 40
DUTY_CYCLE_CENTER = 6

class Car:
    def __init__(self) -> None:
        # Below initializes the variable kit to be our I2C Connected Adafruit Motor HAT.
        self.kit = MotorKit(i2c=board.I2C())

        # Use BCM pin numbering
        servo_pin = 17  # Use BCM pin 17 (equivalent to BOARD pin 11)
        # Set up the PWM for the servo
        self.duty = 2
        GPIO.setup(servo_pin, GPIO.OUT)
        servo = GPIO.PWM(servo_pin, 50)  # 50Hz pulse
        self.servo = servo

        # set up the distance sensor
        #Define the GPIO pins for trigger and echo
        TRIG_PIN = 24  # Trigger pin
        ECHO_PIN = 23  # Echo pin

        self.trig_pin = TRIG_PIN
        self.echo_pin = ECHO_PIN
        # Set up the GPIO pins
        GPIO.setup(TRIG_PIN, GPIO.OUT)
        GPIO.setup(ECHO_PIN, GPIO.IN)

    def stop(self):
        self.kit.motor1.throttle = None
        self.kit.motor2.throttle = None
        self.kit.motor3.throttle = None
        self.kit.motor4.throttle = None
        time.sleep(0.5)
        print("car stopped")

    def move_forward(self):
        # Check the DC motor orientation and provide right values
        self.kit.motor1.throttle = -0.5
        self.kit.motor3.throttle = -0.5

        self.kit.motor2.throttle = 0.5
        self.kit.motor4.throttle = 0.5
        print("moving forward")

    def move_backward(self):
        self.stop()
        self.kit.motor1.throttle = 0.5
        self.kit.motor3.throttle = 0.5

        self.kit.motor2.throttle = -0.5
        self.kit.motor4.throttle = -0.5
        print("moving backward")

    def move_right(self):
        self.stop()
        self.kit.motor1.throttle = None
        self.kit.motor2.throttle = -1
        self.kit.motor3.throttle = None
        self.kit.motor4.throttle = -1
        print("moving right")

    def move_left(self):
        self.stop()
        self.kit.motor1.throttle = -1
        self.kit.motor2.throttle = None
        self.kit.motor3.throttle = -1
        self.kit.motor4.throttle = None
        print("moving left")

    def look_right(self):

        self.stop()
        # Wait a couple of seconds
        time.sleep(2)
        self.servo.ChangeDutyCycle(2.5)
        time.sleep(0.5)
        distance = self.measure_distance()

        self.servo.ChangeDutyCycle(6)
        # return the distance as car need to make a decision
        return distance

    def look_left(self):
        self.stop()
        # Wait a couple of seconds
        time.sleep(2)
        self.servo.ChangeDutyCycle(12.5)
        time.sleep(0.5)

        distance = self.measure_distance()
        self.servo.ChangeDutyCycle(6)
        # return the distance as car need to make a decision
        return distance

    def look_around(self):
        # start PWM running, but with value of 0 (pulse off)
        self.stop()
        # Wait a couple of seconds
        time.sleep(2)

        # Loop for duty values from 2 to 12 (0 to 180 degrees)
        while self.duty <= 12:
            self.servo.ChangeDutyCycle(self.duty)
            time.sleep(0.3)
            self.servo.ChangeDutyCycle(0)
            time.sleep(0.7)
            self.duty = self.duty + 2

        # Wait a couple of seconds
        time.sleep(2)
        self.servo.ChangeDutyCycle(0)

        self.servo.ChangeDutyCycle(7)
        time.sleep(0.5)
        self.servo.ChangeDutyCycle(0)
        time.sleep(1.5)

    def turn_off(self):
        self.stop()
        # Clean things up at the end
        self.servo.ChangeDutyCycle(0)
        time.sleep(1)
        self.servo.stop()
        GPIO.cleanup()
        print("Goodbye")
