import socket
import time

host_addr = '202.238.218.44'
host_port = 3442

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host_addr, host_port))
time.sleep(1)
print("Sending: 'Hello, TCP!")
s.send(b"Hello, TCP!")
data = s.recv(1024)
print(data)
print("Closing connection")
s.close()
