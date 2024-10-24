import RPi.GPIO as GPIO
import time

# Motor Driver Pins
IN1 = 17  # GPIO Pin for Motor A IN1
IN2 = 27  # GPIO Pin for Motor A IN2
ENA = 22  # GPIO Pin for Motor A ENA (PWM)

IN3 = 23  # GPIO Pin for Motor B IN3
IN4 = 24  # GPIO Pin for Motor B IN4
ENB = 25  # GPIO Pin for Motor B ENB (PWM)

# Setup the GPIO mode
GPIO.setmode(GPIO.BCM)

# Setup Motor A GPIO pins
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

# Setup Motor B GPIO pins
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Set up PWM on the ENA and ENB pins (motor speed control)
pwm_a = GPIO.PWM(ENA, 1000)  # 1000 Hz PWM frequency for Motor A
pwm_b = GPIO.PWM(ENB, 1000)  # 1000 Hz PWM frequency for Motor B

# Start PWM with 0% duty cycle (motors off)
pwm_a.start(0)
pwm_b.start(0)

def set_motor_a_speed(speed):
    """Set the speed of motor A (speed: 0 to 100)."""
    pwm_a.ChangeDutyCycle(speed)

def set_motor_b_speed(speed):
    """Set the speed of motor B (speed: 0 to 100)."""
    pwm_b.ChangeDutyCycle(speed)

def run_motor_a(direction, speed):
    """Run Motor A in the given direction and speed."""
    if direction == 'forward':
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    elif direction == 'backward':
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    
    set_motor_a_speed(speed)

def run_motor_b(direction, speed):
    """Run Motor B in the given direction and speed."""
    if direction == 'forward':
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
    elif direction == 'backward':
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
    
    set_motor_b_speed(speed)

def stop_motor_a():
    """Stop Motor A."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    set_motor_a_speed(0)

def stop_motor_b():
    """Stop Motor B."""
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    set_motor_b_speed(0)

try:
    # Example usage
    print("Running Motor A Forward at 50% speed")
    run_motor_a('forward', 50)  # Run Motor A forward at 50% speed
    time.sleep(2)
    
    print("Running Motor B Backward at 75% speed")
    run_motor_b('backward', 75)  # Run Motor B backward at 75% speed
    time.sleep(2)

    print("Stopping both motors")
    stop_motor_a()
    stop_motor_b()
    time.sleep(1)

finally:
    # Cleanup the GPIO pins before exiting
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
