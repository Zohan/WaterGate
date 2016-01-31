import time
import serial
import os

def check_for_trouble(data):
	for i in range(len(data)-2):
		if (data[i] == '4' or data[i] == '5' or data[i] == '6') and \
			(data[i+1] == ' ') and (data[i+2] == '1' or data[i+2] == '2'):

			return True

	return False

ser = serial.Serial(
	port='/dev/ttyAMA0',
	baudrate=19200	)

try:
	trouble = False

	for i in range(10):
		data = ser.read(10)
		trouble |= check_for_trouble(data)
		#time.sleep(0.2)

	if trouble:
		os.system('echo 1 > status.txt')
	else:
		os.system('echo 0 > status.txt')

	ser.close()

except:
	os.system('echo 0 > status.txt')
	ser.close()
