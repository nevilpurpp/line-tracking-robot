import RPi.GPIO as GPIO  
import time  
from functions import *
from definitions import *


def main():
    try:
        #set up functions
        setup()

        #main loop
        while True:
            #perform tasks, and robot logic
            while not L:
                forward()
                
            forward()
            time.sleep(4)
  
    except KeyboardInterrupt:

        GPIO.cleanup()

if __name__ == "__main__":
    main()