
# motor control
LEFT_MOTOR_FORWARD_PIN = 17
LEFT_MOTOR_BACKWARD_PIN = 18
RIGHT_MOTOR_FORWARD_PIN = 22
RIGHT_MOTOR_BACKWARD_PIN = 23
LEFT_MOTOR_PWM_PIN = 27  
RIGHT_MOTOR_PWM_PIN = 24 

#ir sensors configuration
ir1 = 5
ir2 = 6
ir3 = 13
ir4 = 20
ir5 = 19

# Define ultrasonic sensor pins
pingPin = 26
echoPin = 27

# Define motor speed and PID control values
max_speed = 100
set_position = 200
kp = 0.5
kd = 4

# Read digital values from the IR sensor pins
ir1_val = GPIO.input(ir1)
ir2_val = GPIO.input(ir2)
ir3_val = GPIO.input(ir3)
ir4_val = GPIO.input(ir4)
ir5_val = GPIO.input(ir5)

# Check conditions similar to your Arduino code
L = ir1_val == 0 and ir2_val == 0 and ir3_val == 1 and ir4_val == 1 and ir5_val == 1
L2 = ir1_val == 1 and ir2_val == 1 and ir3_val == 0 and ir4_val == 0 and ir5_val == 0

# Print the results for debugging
print("L:", L)
print("L2:", L2)
