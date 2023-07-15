import serial 
import time
import pandas as pd
import json

def conv_json_csv(json_data):
	
	try:
		df = pd.DataFrame.from_dict(json_data, orient='index')
		df.to_csv('data.csv', index_label='key')
		print("Success")
	except AttributeError as a:
		print("Attribute error: ", a)
	# insert a try and except here

ser = serial.Serial('/dev/ttyUSB0', 115200)

while True:
	if ser.in_waiting > 0:
		data = ser.readline().decode().strip()
		# print(type(data)) <class 'str>
		if (data[ :15] == "Received data: "):
			y=len(data[15:])
			ndata=data[15:14+y]
			print(f"{ndata}")
			try:
				x=json.loads(ndata)
				conv_json_csv(x)
			except json.JSONDecodeError as e:
				print("Json decode error", e)
			
		'''else:
			print("wron")'''
		#print(f"{data}")
		
		
		# for future data processing tasks
		
	ser.flush()
	#time.sleep(1)
	
ser.close()


