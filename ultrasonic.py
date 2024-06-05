def measure_distance():
    # Send a 10µs pulse to TRIG to start the measurement
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Wait for the ECHO pin to go high and record the start time
    while GPIO.input(ECHO) == 0:
        start_time = time.time()

    # Wait for the ECHO pin to go low and record the end time
    while GPIO.input(ECHO) == 1:
        end_time = time.time()

    # Calculate the duration of the pulse
    pulse_duration = end_time - start_time

    # Calculate the distance in cm (speed of sound is 34300 cm/s)
    distance = (pulse_duration * 34300) / 2

    return distance