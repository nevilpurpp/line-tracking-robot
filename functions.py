from definitions import *

def setup():
  """
  This function sets up the GPIO pins and prepares the motors and sensors.
  """
  GPIO.setmode(GPIO.BCM)  

  # Set motor control pins as output
  GPIO.setmode(GPIO.BCM)

  GPIO.setup(LEFT_MOTOR_FORWARD_PIN, GPIO.OUT)
  GPIO.setup(LEFT_MOTOR_BACKWARD_PIN, GPIO.OUT)
  GPIO.setup(RIGHT_MOTOR_FORWARD_PIN, GPIO.OUT)
  GPIO.setup(RIGHT_MOTOR_BACKWARD_PIN, GPIO.OUT)

# Setup PWM pins
  GPIO.setup(LEFT_MOTOR_PWM_PIN, GPIO.OUT)
  GPIO.setup(RIGHT_MOTOR_PWM_PIN, GPIO.OUT)


  # Set sensor pins as input
  for i in range(ir1, ir5 + 1):  # Loop through all IR sensor pins
       GPIO.setup(i, GPIO.IN)

  # Set ultrasonic sensor pins
  GPIO.setup(pingPin, GPIO.OUT)
  GPIO.setup(echoPin, GPIO.IN)

def forward():
  """
  This function implements line tracking logic using sensor readings and PID control.
  """
  global last_p, sum_i, sum_w

  sum_w = 0
  sum_i = 0
  error = 0
  d = 0

  # Read sensor values
  for i in range(5):
    sensor[i] = GPIO.input(ir1 + i)
    sum_w += sensor[i] * (i * 100)  # Calculate weighted sum based on sensor position
    sum_i += sensor[i]

  # Calculate current robot position based on sensor readings
  cur_position = sum_w / sum_i

  # Calculate proportional error (difference between desired and current position)
  p = set_position - cur_position

  # Calculate derivative error (change in error over time)
  d = p - last_p
  last_p = p

  # Calculate total error using PID constants
  error = p * kp + d * kd

  # Print error value for debugging (optional)
  print("Error:", error)

  # Determine motor speeds based on error
  if error < 0:
    left_wheel = max_speed
    right_wheel = max_speed - error
  else:
    right_wheel = max_speed
    left_wheel = max_speed + error

  # Set motor directions and speeds
  GPIO.output(in1, GPIO.HIGH)
  GPIO.output(in2, GPIO.LOW)
  GPIO.output(in3, GPIO.HIGH)
  GPIO.output(in4, GPIO.LOW)
  GPIO.output(RIGHT_MOTOR_PWM_PIN, right_wheel)
  GPIO.output(LEFT_MOTOR_PWM_PIN, left_wheel)


def forward_cross():
    """ 
    this function implements a junction counter """


def right_turn():
    """
    this function implements a right turn"""

def left_turn():
    """
    this function implements a left turn"""  

def reverse():
    """
    this function implements a reverse logic"""


  