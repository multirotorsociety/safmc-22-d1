# Import Module #
import guli
#import keyboard
from pynput.keyboard import Key, Controller, Listener
# Initialising Variables #
keyboard = Controller()
j=0
yaw = 0
accend = 0
em = 0

def macropadMapping(): # Events Listener Format
	# Collect Events until Released
	with Listener(on_press = on_press, on_release = on_release) as listener:
		listener.join()

def on_press(key): # HOLDING THE KEY will continue triggering this on_press function!
	#print('{0} pressed'.format(key))print(Ascend,EM)
	global yaw,accend
	try:
		if key.char == 'a': # Yaw Left
			if not yaw == 1:
				yaw = 1
				guli.GuliVariable("y").setValue(yaw)
				print("Yaw Left")
		elif key.char == 's': # Yaw Right
			if not yaw == -1:
				yaw = -1
				guli.GuliVariable("y").setValue(yaw)
				print("Yaw Right")
		
		if key.char == 'd': # Ascend
			if not accend == 1:
				accend = 1
				guli.GuliVariable("a").setValue(accend)
				print("Ascend")
		elif key.char == 'h': # Descend
			if not accend == -1:
				accend = -1
				guli.GuliVariable("a").setValue(accend)
				print("Descend")
	except:
		pass
	
	

def on_release(key):
	global yaw,accend,em
	try:
	#print('{0} release'.format(key))
		if key.char == 'h' or key.char == 'd': # Hover
			accend = 0
			guli.GuliVariable("a").setValue(accend)
			print("height 0")
		if key.char == 'a' or key.char == 's': # Hover
			yaw = 0
			guli.GuliVariable("y").setValue(yaw)
			print("yaw 0")
		if key.char == 'f':
			em = 1
			guli.GuliVariable("EM").setValue(em)
			print("Electromagnet Activated!")
		
		if key.char == 'g':
			em = 0
			guli.GuliVariable("EM").setValue(em)
			print("Electromagnet Deactivated!")
		if key == Key.esc:
			# Stop Listener
			return False
	except:
		pass


#=====================================

#  Main Program Function

#=====================================

def main():
	guli.GuliVariable("EM").setValue(1.0)
	guli.GuliVariable("y").setValue(0.0)
	guli.GuliVariable("a").setValue(0.0)
	guli.GuliVariable("p").setValue(0.0)
	guli.GuliVariable("r").setValue(0.0)
	print("reading data from macropad")
	macropadMapping()

if __name__ == "__main__":
	main() # Execute Main Program
