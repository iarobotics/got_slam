import serial, sys

port = "/dev/ttyACM0"

if (len(sys.argv) != 2):
	print("Usage: python ReadSerial.py port")
	print("Using default port: /dev/ttyACM0")
else:
	port = sys.argv[1]

serialFromArduino = serial.Serial(port,9600)
serialFromArduino.flushInput()

while True:
	if (serialFromArduino.inWaiting() > 0):
		input = serialFromArduino.read(1)
		print(ord(input))
		print(str(ord(input)) + " = the ASCII character " + input +".")
