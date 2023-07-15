import serial 
import time
import pandas as pd
import json

def conv_json_csv(json_data):
	
	try:
		df = pd.DataFrame(json_data)
		df.to_csv('data.csv', mode='a', header=True, index_label= "Address")
		print("Success")
	except AttributeError as ae:
		print("Attribute error: ", ae)
	# insert a try and except here
	
while True:
	try:
		ser = serial.Serial('/dev/ttyUSB0', 115200)
		break
	except serial.serialutil.SerialException:
		print("No device connected! Please check! \n \n")
		time.sleep(10)
	except BrokenPipeError:
		print("No device connected! Please check! \n \n")
		time.sleep(10)
	except termios.error:
		print("No device connected! Please check! \n \n")
		time.sleep(10)
	except OSError:
		print("No device connected! Please check! \n \n")
		time.sleep(10)

while True:
	if ser.in_waiting > 0:
		data = ser.readline().decode().strip()
		# print(type(data)) <class 'str'>
		if (data[ :15] == "Received data: "):
			y=len(data[15:])
			ndata=data[15:14+y]
			print(f"{ndata}")
			try:
				x=json.loads(ndata)
				print(x)
				conv_json_csv(x)
			except json.JSONDecodeError as e:
				print("Json decode error", e)
			
		'''else:
			print("wron")'''
		#print(f"{data}")
		
		
	ser.flush()
	#time.sleep(1)
	
ser.close()


