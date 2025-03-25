import RPi.GPIO as GPIO
import time

# GPIO Pins
SERVO_PWM = 12              # Servo PWM control (default: 12 for hardware PWM)

# Servo Variables
servo_pwm_freq = 50         # PWM frequency for servo (e.g. 50Hz)

servo_pwm_duty_lower = 1.5   # value to scale down to 0 degree (0-255) (default value: 1.5)
servo_pwm_duty_upper = 8.4   # value to scale up to 180 degree (0-255) (default value: 8.4)

	# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PWM, GPIO.OUT)
servo_pwm = GPIO.PWM(SERVO_PWM, servo_pwm_freq)

servo_pwm.start(0)

def ServoMove(angle):
    servo_duty = (angle / 180) * (servo_pwm_duty_upper - servo_pwm_duty_lower) + servo_pwm_duty_lower #scaling
    servo_pwm.ChangeDutyCycle(servo_duty)

print("Start")

try:
    while True:
        angle = int(input('Angle to move servo to (0-180):\n'))
        if angle <= 180 and angle >= 0:
            print('Moving to: ' + str(angle) + ' degrees')
            ServoMove(angle)
            time.sleep(0.2)

except KeyboardInterrupt:
    # End the script and exit
    pwm.stop()
    GPIO.cleanup()