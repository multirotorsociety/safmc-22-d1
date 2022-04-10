"""
Inputs: PLG 2.4G Wireless Numeric Keypad
		IMU MPU6050 Gyroscope Sensor 
Arduino Setup:  (Transmitter V1) Arduino Nano, NRF24L01+ Radio Module, MPU6050
				(Receiver V1) Arduino UNO, NRF24L01+ Radio Module, OLED Screen; Serial Communication via USB
				(Transmitter V2) ESP32 TTGO, MPU6050
				(Receiver V2) Python Script <- WiFi for ESP32, Python Script <- Keypad Input via USB Dongle 
"""

# Import Module #
import serial, time, guli
#import keyboard
from pynput.keyboard import Controller

# Initialising Variables #
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)
keyboard = Controller()
p=0
r=0
guli.GuliVariable("EM").setValue(1.0)
guli.GuliVariable("y").setValue(0.0)
guli.GuliVariable("a").setValue(0.0)
guli.GuliVariable("p").setValue(0.0)
guli.GuliVariable("r").setValue(0.0)

#=====================================

#  Function Definitions

#=====================================

def readArduinoInputs():
	global p,r
	data = arduino.readline()
	# Data Processing #
	stateMode = data.decode().strip()   # F,B,L,R,N,1(Rear Left),3(Rear right),7(Front left),9(Front Right)
	if stateMode == 'F' or stateMode == '7' or stateMode == '9':
		np = 1
	elif stateMode == 'B' or stateMode == '1' or stateMode == '3':
		np = -1
	if stateMode == 'L' or stateMode == '1' or stateMode == '7':
		nr = 1
	elif stateMode == 'R' or stateMode == '3' or stateMode == '9':
		nr = -1
	if stateMode == 'N':
		np=0
		nr=0
	if not np == p:
		print("pitch" +  str(np))
		guli.GuliVariable("p").setValue(np)
		p = np
	if not nr == r:
		print("roll" +  str(np))
		guli.GuliVariable("r").setValue(nr)
		r = nr
#=====================================

#  Main Program Function

#=====================================

def main():
	print("Receiving Datas from Arduino...")
	while True:
		readArduinoInputs()
		# End of Program #

if __name__ == "__main__":
	main() # Execute Main Program
