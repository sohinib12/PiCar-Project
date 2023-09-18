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
        # Setup the PWM for the servo
        self.duty = 2
        GPIO.setup(servo_pin, GPIO.OUT)
        servo = GPIO.PWM(servo_pin, 50)  # 50Hz pulse
        self.servo = servo

        # setup the distance sensor
        # Define the GPIO pins for trigger and echo
        TRIG_PIN = 24  # Trigger pin
        ECHO_PIN = 23  # Echo pin

        self.trig_pin = TRIG_PIN
        self.echo_pin = ECHO_PIN
        # Setup the GPIO pins
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

    def measure_distance(self):
        # Send a short pulse to trigger the ultrasonic sensor
        GPIO.output(self.trig_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, GPIO.LOW)

        pulse_start_time = time.time()
        pulse_end_time = time.time()

        # Listen to the echo response
        while GPIO.input(self.echo_pin) == 0:
            pulse_start_time = time.time()

        while GPIO.input(self.echo_pin) == 1:
            pulse_end_time = time.time()

        # Calculate the duration of the echo
        pulse_duration = pulse_end_time - pulse_start_time

        # Speed of sound at sea level is approximately 343 meters/second
        # Divide by 2 because the sound travels to the object and back
        distance = (pulse_duration * 34300) / 2

        return distance

    def servo_start(self):
        self.servo.start(0)
        time.sleep(1)

    def servo_move_left(self):
        # Decrease the duty cycle to move the servo to the left
        self.servo.ChangeDutyCycle(DUTY_CYCLE_CENTER - 1.0)
        time.sleep(1)  # Adjust the delay as needed

    def servo_move_right(self):
        # Increase the duty cycle to move the servo to the right
        self.servo.ChangeDutyCycle(DUTY_CYCLE_CENTER + 1.0)
        time.sleep(1)  # Adjust the delay as needed

    def check_space(self):
        self.stop()
        time.sleep(0.5)
        self.move_backward()
        time.sleep(1.5)
        l1 = self.look_left()
        l2 = self.look_right()
        print(f'left side is clear {l1:.2f} cm')
        print(f'right side is clear {l2:.2f} cm')
        # turn to the side where it can find some space
        if l1 > l2 and l1 > THRESHOLD_DISTANCE:
            self.move_left()
            time.sleep(1)
            self.stop()

        elif l2> l1 and l2 > THRESHOLD_DISTANCE:
            self.move_right()
            time.sleep(1)
            self.stop()
        else:
            l3 = self.look_left()
            l4 = self.look_right()
            if l3 > l4:
                self.move_left()
                time.sleep(1)
            else:
                self.move_right()
                time.sleep(1)

car = Car()
car.servo_start()

try:
    while True:
        distance = car.measure_distance()  # Measure the distance
        print(f'All clear. distance is {distance:.2f} cm')
        if distance < THRESHOLD_DISTANCE:
            car.stop()  # Stop if something is too close
            car.check_space()
        else:
            car.move_forward()  # Move forward if the path is clear

        time.sleep(0.5)

except KeyboardInterrupt:
    car.stop()
    time.sleep(1)
    car.turn_off()
