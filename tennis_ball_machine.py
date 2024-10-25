
import tkinter as tk
from tkinter import ttk
import RPi.GPIO as GPIO
import time

# GPIO setup
GPIO.setmode(GPIO.BCM)

# Motor Driver Pins
IN1, IN2, ENA = 17, 27, 22  # Motor A
IN3, IN4, ENB = 23, 24, 25  # Motor B

# Servo Pins
FEEDER_SERVO_PIN = 18      # Servo to control ball feed
HORIZ_SERVO_PIN = 12       # Servo to control horizontal angle
VERT_SERVO_PIN = 13        # Servo to control vertical angle

# Initialize DC Motor pins
GPIO.setup([IN1, IN2, ENA, IN3, IN4, ENB], GPIO.OUT)
pwm_a = GPIO.PWM(ENA, 1000)  # PWM for Motor A
pwm_b = GPIO.PWM(ENB, 1000)  # PWM for Motor B
pwm_a.start(0)
pwm_b.start(0)

# Initialize Servo pins
GPIO.setup(FEEDER_SERVO_PIN, GPIO.OUT)
GPIO.setup(HORIZ_SERVO_PIN, GPIO.OUT)
GPIO.setup(VERT_SERVO_PIN, GPIO.OUT)
feeder_servo = GPIO.PWM(FEEDER_SERVO_PIN, 50)
horiz_servo = GPIO.PWM(HORIZ_SERVO_PIN, 50)
vert_servo = GPIO.PWM(VERT_SERVO_PIN, 50)
feeder_servo.start(0)
horiz_servo.start(0)
vert_servo.start(0)

# Function to control motor speed and direction
def run_motor(motor, speed, direction):
    if motor == 'A':
        GPIO.output(IN1, GPIO.HIGH if direction == 'forward' else GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW if direction == 'forward' else GPIO.HIGH)
        pwm_a.ChangeDutyCycle(speed)
    elif motor == 'B':
        GPIO.output(IN3, GPIO.HIGH if direction == 'forward' else GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW if direction == 'forward' else GPIO.HIGH)
        pwm_b.ChangeDutyCycle(speed)

def stop_motors():
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)

# Function to control servo angle
def set_servo_angle(servo, angle):
    duty_cycle = 2 + (angle / 18)
    if servo == 'feeder':
        feeder_servo.ChangeDutyCycle(duty_cycle)
    elif servo == 'horizontal':
        horiz_servo.ChangeDutyCycle(duty_cycle)
    elif servo == 'vertical':
        vert_servo.ChangeDutyCycle(duty_cycle)
    time.sleep(0.3)
    feeder_servo.ChangeDutyCycle(0)

# Initialize the Tkinter GUI
root = tk.Tk()
root.title("Tennis Ball Throwing Machine")
root.geometry("800x480")  # Adjust to touchscreen resolution

# Motor control frame
motor_frame = ttk.LabelFrame(root, text="Motor Control")
motor_frame.pack(padx=10, pady=10, fill="x")

# Motor A Speed Control
ttk.Label(motor_frame, text="Motor A Speed:").grid(row=0, column=0, padx=5, pady=5)
speed_a = tk.IntVar()
speed_a_slider = tk.Scale(motor_frame, from_=0, to=100, orient="horizontal", variable=speed_a)
speed_a_slider.grid(row=0, column=1, padx=5, pady=5)
direction_a = tk.StringVar(value="forward")
ttk.Radiobutton(motor_frame, text="Forward", variable=direction_a, value="forward").grid(row=0, column=2)
ttk.Radiobutton(motor_frame, text="Backward", variable=direction_a, value="backward").grid(row=0, column=3)

# Motor B Speed Control
ttk.Label(motor_frame, text="Motor B Speed:").grid(row=1, column=0, padx=5, pady=5)
speed_b = tk.IntVar()
speed_b_slider = tk.Scale(motor_frame, from_=0, to=100, orient="horizontal", variable=speed_b)
speed_b_slider.grid(row=1, column=1, padx=5, pady=5)
direction_b = tk.StringVar(value="forward")
ttk.Radiobutton(motor_frame, text="Forward", variable=direction_b, value="forward").grid(row=1, column=2)
ttk.Radiobutton(motor_frame, text="Backward", variable=direction_b, value="backward").grid(row=1, column=3)

# Control Motor button
def set_motor():
    run_motor('A', speed_a.get(), direction_a.get())
    run_motor('B', speed_b.get(), direction_b.get())

ttk.Button(motor_frame, text="Set Motor Speeds", command=set_motor).grid(row=2, column=0, columnspan=4, pady=10)

# Servo control frame
servo_frame = ttk.LabelFrame(root, text="Servo Control")
servo_frame.pack(padx=10, pady=10, fill="x")

# Feeder Servo Control
ttk.Label(servo_frame, text="Feeder Servo Angle:").grid(row=0, column=0, padx=5, pady=5)
feeder_angle = tk.IntVar()
feeder_slider = tk.Scale(servo_frame, from_=0, to=180, orient="horizontal", variable=feeder_angle)
feeder_slider.grid(row=0, column=1, padx=5, pady=5)

# Horizontal Servo Control
ttk.Label(servo_frame, text="Horizontal Servo Angle:").grid(row=1, column=0, padx=5, pady=5)
horizontal_angle = tk.IntVar()
horizontal_slider = tk.Scale(servo_frame, from_=0, to=180, orient="horizontal", variable=horizontal_angle)
horizontal_slider.grid(row=1, column=1, padx=5, pady=5)

# Vertical Servo Control
ttk.Label(servo_frame, text="Vertical Servo Angle:").grid(row=2, column=0, padx=5, pady=5)
vertical_angle = tk.IntVar()
vertical_slider = tk.Scale(servo_frame, from_=0, to=180, orient="horizontal", variable=vertical_angle)
vertical_slider.grid(row=2, column=1, padx=5, pady=5)

# Control Servo button
def set_servo():
    set_servo_angle('feeder', feeder_angle.get())
    set_servo_angle('horizontal', horizontal_angle.get())
    set_servo_angle('vertical', vertical_angle.get())

ttk.Button(servo_frame, text="Set Servo Angles", command=set_servo).grid(row=3, column=0, columnspan=2, pady=10)

# Stop Motors button
ttk.Button(root, text="Stop Motors", command=stop_motors, style="Danger.TButton").pack(pady=10)

# Cleanup GPIO on exit
def on_closing():
    stop_motors()
    pwm_a.stop()
    pwm_b.stop()
    feeder_servo.stop()
    horiz_servo.stop()
    vert_servo.stop()
    GPIO.cleanup()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
