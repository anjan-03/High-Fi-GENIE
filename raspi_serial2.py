import serial 
import time
import pandas as pd
import json
from pushbullet import Pushbullet
import csv
global start_node
def conv_json_csv(json_data):
	
	try:
		df = pd.DataFrame(json_data)
		df.to_csv('data.csv', mode='w', header=True, index_label= "Address")
		print("Success")
	except AttributeError as ae:
		print("Attribute error: ", ae)
	# insert a try and except here

def func(start_node):
    API_KEY1="o.ySCQXKRUFeQBLpbYYmnpcI2rc9Gm9PRJ" #fire.iot2023@gmail.com   fire_iot
    API_KEY2="o.TqF9EFmwGWAxptnZCKOMmEseFQOwNrJ4" #fire.iot2003@gmail.com
    API_KEY3="o.851PvlIpYnaeKtBizM1S0HVBy4oxrxme" #fire.iot79@gmail.com
    API_KEY4="o.PAAkqePKFVUSqEPvcmrqpSVKZmuYlJrR" #fire.iot3435@gmail.com
    file="fire.txt"
    with open(file,mode='r') as f:
        text=f.read()
    if(start_node=='A'):
        pb=Pushbullet(API_KEY1)
    elif(start_node=='B'):
        pb=Pushbullet(API_KEY3)
    elif(start_node=='C'):
        pb=Pushbullet(API_KEY2)
    elif(start_node=='D'):
        pb=Pushbullet(API_KEY4)
    push=pb.push_note('Escape through gate',text)
    f.close()

def func2(start_node):
	with open("data.csv",mode='r') as csv_file:
		reader = csv.reader(csv_file)
		lines = list(reader)
		for i in range(len(lines)):
			if (i+1) % 2 == 0:
				decimal_value = lines[i][-1]
				if(float(decimal_value)>35.00):
					func(start_node)
	csv_file.close()

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
		if (data[ :15] == "Received Data: "):
			y=len(data[15:])
			ndata=data[15:14+y]
			print(f"{ndata}")
			try:
				x=json.loads(ndata)
				print(x)
				conv_json_csv(x)
			except json.JSONDecodeError as e:
				print("Json decode error", e)
			
		elif (data[ :15] == "Starting node: "):
			yy=len(data[15:])
			ndata=data[15:14+y]
			start_node=ndata
			print(f"Starting node: {start_node}")
			func2(start_node)
		
		
	ser.flush()
	
	
ser.close()