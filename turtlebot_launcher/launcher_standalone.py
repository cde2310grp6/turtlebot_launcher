import RPi.GPIO as GPIO
import time

# GPIO Pins
FLYWHEEL_PWM = 18           # Single PWM control for both motors (wired in parallel)
SERVO_PWM = 12              # Servo PWM control (default: 12 for hardware PWM)

# Motor Variables
motor_pwm_freq = 1000       # PWM frequency for motor (e.g. 1000Hz)
motor_pwm_on_duty = 70      # Duty cycle for motor in %. (e.g. 70)

# Servo Variables
servo_pwm_freq = 50         # PWM frequency for servo (e.g. 50Hz)
servo_pwm_duty_lower = 1.5   # value to scale down to 0 degree (0-255) (default value: 1.5)
servo_pwm_duty_upper = 8.4   # value to scale up to 180 degree (0-255) (default value: 8.4)

servo_launch_angle = 90     # Angle to set servo to launch ball (0-180)
servo_reset_angle = 40       # Angle to reset servo to (0-180)
flywheel_spinup_time = 1    # Time needed to spin up flywheels
flywheel_spindown_time = 1  # Time needed to slow down flywheels
flywheel_launch_time = 0.5  # Time needed to launch ball

flag = 0

	# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(FLYWHEEL_PWM, GPIO.OUT)
GPIO.setup(SERVO_PWM, GPIO.OUT)

flywheel_pwm = GPIO.PWM(FLYWHEEL_PWM, motor_pwm_freq)
servo_pwm = GPIO.PWM(SERVO_PWM, servo_pwm_freq)

flywheel_pwm.start(0)  # Initialize PWM at 0
servo_pwm.start(0)

#self.get_logger().info("TurtleBot Launcher service is ready!")

def ServoMove(angle):
    servo_duty = (angle / 180) * (servo_pwm_duty_upper - servo_pwm_duty_lower) + servo_pwm_duty_lower
    servo_pwm.ChangeDutyCycle(servo_duty)

def FlywheelStart():
    flywheel_pwm.ChangeDutyCycle(motor_pwm_on_duty)

def FlywheelStop():
    flywheel_pwm.ChangeDutyCycle(0)

def LaunchBall():
    # Reset servo just in case it's not at 0 degrees
    print("Resetting servo...")
    ServoMove(servo_reset_angle)

    # Start flywheel motors
    FlywheelStart()

    # Wait for motors to reach speed
    print("Spinning up flywheels...")
    time.sleep(flywheel_spinup_time)

    # Move servo to launch the ball
    print("Launching ball with servo...")
    ServoMove(servo_launch_angle)
    time.sleep(flywheel_launch_time)  # Delay to allow servo to move

    # Stop flywheels after launch
    print("Stopping flywheels...")
    FlywheelStop()
    time.sleep(flywheel_spindown_time)

    # Reset servo to 0 degrees
    ServoMove(servo_reset_angle)
    print("Resetting servo...")
    time.sleep(1)  # Delay to allow servo to move

LaunchBall()
LaunchBall()
time.sleep(1)
LaunchBall()

# Clean up GPIO
flywheel_pwm.stop()
servo_pwm.stop()
GPIO.cleanup()
