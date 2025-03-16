#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pigpio
from std_srvs.srv import Trigger  #Simple service with no request arguments 

# GPIO Pins
FLYWHEEL_PWM = 12           # Single PWM control for both motors (wired in parallel)
SERVO_PWM = 18              # Servo PWM control

# Motor Variables
motor_pwm_freq = 1000       # 1kHz PWM for motor
motor_pwm_on_duty = 70          # 70% duty cycle for motor

# Servo Variables
servo_pwm_freq = 50         # 50Hz PWM for servo
servo_pwm_duty_lower = 15       #value to scale down to 0 degree
servo_pwm_duty_upper = 38       #value to scale up to 180 degree

servo_launch_angle = 50     # Angle to launch ball
servo_reset_angle = 0       # Angle to reset servo
flywheel_spinup_time = 1    # Time to spin up flywheels
flywheel_spindown_time = 1  # Time to slow down flywheels
flywheel_launch_time = 0.5  # Time to launch ball

# Conversion
motor_pwm_on_duty_scaled = motor_pwm_on_duty/100*255

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Could not connect to pigpio daemon")

def ServoMove(angle):
    duty = (angle/180)*(servo_pwm_duty_upper-servo_pwm_duty_lower)+servo_pwm_duty_lower
    pi.set_PWM_dutycycle(SERVO_PWM, duty) 

def FlywheelStart():
    pi.set_PWM_dutycycle(FLYWHEEL_PWM, motor_pwm_on_duty_scaled)  # Motors on

def FlywheelStop():
    pi.set_PWM_dutycycle(FLYWHEEL_PWM, 0)  # Motors off
    
class TurtleBotLauncher(Node):
    def __init__(self):
        super().__init__('turtlebot_launcher_service')

        # Create a ROS 2 service to trigger the launcher
        self.srv = self.create_service(Trigger, 'launch_ball', self.launch_ball_callback)

        # Set up motor PWM (initially off)
        pi.set_PWM_frequency(FLYWHEEL_PWM, motor_pwm_freq)  # Set Motor PWM frequency
        pi.set_PWM_frequency(SERVO_PWM,servo_pwm_freq) # Set Servo PWM frequency

        self.get_logger().info("TurtleBot Launcher service is ready!")

    def launch_ball_callback(self, request, response):
        self.get_logger().info("Received request to launch ball!")

        # Reset servo just in case it's not at 0 degrees
        ServoMove(servo_reset_angle)

        # Start flywheel motors (set PWM speed, e.g., 70%)
        FlywheelStart()

        # Wait for motors to reach speed
        self.get_logger().info("Spinning up flywheels...")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=flywheel_spinup_time))

        # Move servo to launch the ball
        self.get_logger().info("Launching ball with servo...")
        ServoMove(servo_launch_angle)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=flywheel_launch_time)) # delay to allow ball to launch

        # Stop flywheels after launch
        FlywheelStop()
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=flywheel_spindown_time))

        # Reset servo to 0 degrees
        ServoMove(servo_reset_angle)

        response.success = True
        response.message = "Ball launched!"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotLauncher()
    rclpy.spin(node)
    pi.stop()  # Cleanup PWM on shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()