# TURN WATER PUMP OFF
# by Lourdes Morales
#
# Reference: https://www.raspberrypi.org/learning/python-quick-reaction-game/worksheet/
#
# First, import module / library needed to control 
# the GPIO pins on the Raspberry Pi

import RPi.GPIO as GPIO

# Make sure the GPIO pins are ready

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set up the pin that the water pump connects to on the Raspberry Pi 
# as an output. 

water_pump_1 = 21 #--------PIN NUMBER FOR WATER PUMP------------------
water_pump_2 = 20

GPIO.setup(water_pump_1, GPIO.OUT)
GPIO.setup(water_pump_2, GPIO.OUT)

# Next, turn water_pump off. 
# 1 represents ON and 0 represents OFF:

GPIO.output(water_pump_1, 0)
GPIO.output(water_pump_2, 0)

GPIO.cleanup()

# print 'Success'; 