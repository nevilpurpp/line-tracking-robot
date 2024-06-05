import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

#camera set up
cap = cv2.VideoCapture(0)
cap.set(3, 160)
cap.set(4, 120)

#motor  control pins
LEFT_MOTOR_FORWARD_PIN = 17
LEFT_MOTOR_BACKWARD_PIN = 18
RIGHT_MOTOR_FORWARD_PIN = 22
RIGHT_MOTOR_BACKWARD_PIN = 23
LEFT_MOTOR_PWM_PIN = 27  # PWM pin for left motor
RIGHT_MOTOR_PWM_PIN = 24  # PWM pin for right motor

#gpio pins for TRIG and ECHO
TRIG = 25
ECHO =26

# setup GPIO mode
GPIO.setmode(GPIO.BCM)

GPIO.setup(LEFT_MOTOR_FORWARD_PIN, GPIO.OUT)
GPIO.setup(LEFT_MOTOR_BACKWARD_PIN, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_FORWARD_PIN, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_BACKWARD_PIN, GPIO.OUT)

# Setup PWM pins
GPIO.setup(LEFT_MOTOR_PWM_PIN, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_PWM_PIN, GPIO.OUT)


# Initialize PWM
left_motor_pwm = GPIO.PWM(LEFT_MOTOR_PWM_PIN, 1000)  # 1kHz frequency
right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_PWM_PIN, 1000)  # 1kHz frequency
left_motor_pwm.start(0)
right_motor_pwm.start(0)

def set_motor_speed(left_speed, right_speed):
    left_motor_pwm.ChangeDutyCycle(left_speed)
    right_motor_pwm.ChangeDutyCycle(right_speed)

def move_forward(left_speed, right_speed):
    GPIO.output(LEFT_MOTOR_FORWARD_PIN, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD_PIN, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_FORWARD_PIN, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD_PIN, GPIO.HIGH)
    set_motor_speed(left_speed, right_speed)

def move_backward(left_speed, right_speed):
    GPIO.output(LEFT_MOTOR_FORWARD_PIN, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_BACKWARD_PIN, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD_PIN, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_BACKWARD_PIN, GPIO.LOW)
    set_motor_speed(left_speed, right_speed)

def turn_left(speed):
    GPIO.output(LEFT_MOTOR_FORWARD_PIN, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD_PIN, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD_PIN, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_BACKWARD_PIN, GPIO.LOW)
    set_motor_speed(0, speed)

def turn_right(speed):
    GPIO.output(LEFT_MOTOR_FORWARD_PIN, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_BACKWARD_PIN, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD_PIN, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD_PIN, GPIO.LOW)
    set_motor_speed(speed, 0)

def stop():
    GPIO.output(LEFT_MOTOR_FORWARD_PIN, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD_PIN, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD_PIN, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD_PIN, GPIO.LOW)
    set_motor_speed(0, 0)

while True:

    ret, frame = cap.read() # reads a frame from the camera and stores return value (ret) and (frame)

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    low_hsv = np.uint8([0, 0, 220])  # Lower bound (any Hue, low Saturation, high Value)
    high_hsv = np.uint8([255, 255, 255]) # Upper bound (any Hue, any Saturation, max Value)
    mask = cv2.inRange(hsv_frame, low_hsv, high_hsv)

   
    contours, hierarchy = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)
    if len(contours) > 0 :
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] !=0 :
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print("CX : "+str(cx)+"  CY : "+str(cy))
             #Control the robot based on the centroid's X position
            if cx >= 120 :
                print("Turn Left")
                turn_left()
            if cx < 120 and cx > 40 :
                print("On Track!")
                move_forward()
            if cx <=40 :
                print("Turn Right")
                turn_right()
            cv2.circle(frame, (cx,cy), 5, (255,255,255), -1)
    else :
        print("I don't see the line")
        stop()
        #GPIO.output(in1, GPIO.LOW)
        #GPIO.output(in2, GPIO.LOW)
        #GPIO.output(in3, GPIO.LOW)
        #GPIO.output(in4, GPIO.LOW)
    cv2.drawContours(frame, c, -1, (0,255,0), 1)
    cv2.imshow("Mask",mask)
    cv2.imshow("Frame",frame)
    if cv2.waitKey(1) & 0xff == ord('q'):   # 1 is the time in ms
        stop()
        break
cap.release()
cv2.destroyAllWindows()

