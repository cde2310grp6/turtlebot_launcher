#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_srvs.srv import Trigger  # Simple service with no request arguments
import time

# GPIO Pins
FLYWHEEL_PWM = 18           # Single PWM control for both motors (wired in parallel)
SERVO_PWM = 12              # Servo PWM control

# Motor Variables
motor_pwm_freq = 1000       # PWM frequency for motor (e.g. 1000Hz)
motor_pwm_on_duty = 70      # Duty cycle for motor in %. (e.g. 70)

# Servo Variables
servo_pwm_freq = 50         # PWM frequency for servo (e.g. 50Hz)
servo_pwm_duty_lower = 1.5   # value to scale down to 0 degree (0-255) (default value: 1.5)
servo_pwm_duty_upper = 8.4   # value to scale up to 180 degree (0-255) (default value: 8.4)

servo_launch_angle = 90     # Angle to set servo to launch ball (0-180)
servo_reset_angle = 40      # Angle to reset servo to (0-180)
flywheel_spinup_time = 1    # Time needed to spin up flywheels
flywheel_spindown_time = 1  # Time needed to slow down flywheels
flywheel_launch_time = 0.5  # Time needed to launch ball

class TurtleBotLauncher(Node):
    def __init__(self):
        super().__init__('turtlebot_launcher_service')

        # Create a ROS 2 service to trigger the launcher
        self.srv = self.create_service(Trigger, 'launch_ball', self.launch_ball_callback)

        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(FLYWHEEL_PWM, GPIO.OUT)
        GPIO.setup(SERVO_PWM, GPIO.OUT)

        self.flywheel_pwm = GPIO.PWM(FLYWHEEL_PWM, motor_pwm_freq)
        self.servo_pwm = GPIO.PWM(SERVO_PWM, servo_pwm_freq)

        self.flywheel_pwm.start(0)  # Initialize PWM at 0
        self.servo_pwm.start(0)

        self.get_logger().info("TurtleBot Launcher service is ready!")

    def ServoMove(self, angle):
        servo_duty = (angle / 180) * (servo_pwm_duty_upper - servo_pwm_duty_lower) + servo_pwm_duty_lower
        self.servo_pwm.ChangeDutyCycle(servo_duty)

    def FlywheelStart(self):
        self.flywheel_pwm.ChangeDutyCycle(motor_pwm_on_duty)

    def FlywheelStop(self):
        self.flywheel_pwm.ChangeDutyCycle(0)

    def LaunchBall(self):
        # Reset servo just in case it's not at 0 degrees
        self.ServoMove(servo_reset_angle)

        # Start flywheel motors
        self.FlywheelStart()

        # Wait for motors to reach speed
        time.sleep(flywheel_spinup_time)

        # Move servo to launch the ball
        self.ServoMove(servo_launch_angle)
        time.sleep(flywheel_launch_time)  # Delay to allow servo to move

        # Stop flywheels after launch
        self.FlywheelStop()
        time.sleep(flywheel_spindown_time)

        # Reset servo to 0 degrees
        self.ServoMove(servo_reset_angle)
        time.sleep(1)  # Delay to allow servo to move
        self.servo_pwm.ChangeDutyCycle(0) # Turn off servo pwm to prevent jitter

    def launch_ball_callback(self, request, response):
        self.get_logger().info("Received request to launch 3 balls!")
        self.LaunchBall()
        response.message = "Ball 1 launched!"
        self.LaunchBall()
        response.message = "Ball 2 launched!"
        time.sleep(1)
        self.LaunchBall()
        response.message = "Ball 3 launched!"
        response.success = True
        return response

    def cleanup(self):
        self.flywheel_pwm.stop()
        self.servo_pwm.stop()
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
