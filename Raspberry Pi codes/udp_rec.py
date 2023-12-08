import socket

UDP_IP = "172.16.20.15"  # check ip address by running ifconfig in rpi terminal (wlan0 ipv4)
UDP_PORT=1234

sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.bind((UDP_IP,UDP_PORT))

while True:
	data,addr = sock.recvfrom(1024)
	print(f"Received message: {data.decode()} from {addr}")