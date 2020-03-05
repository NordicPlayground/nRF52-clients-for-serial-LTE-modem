import socket

host_addr = '202.238.218.44'
host_port = 3442
host = (host_addr, host_port)
local_addr = '5.189.130.26'
local_port = 3442
local = (local_addr, local_port)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(local)
print("Sending: 'Hello, UDP!")
s.sendto(b"Hello, UDP!", host)
data, address = s.recvfrom(1024)
print(data)
print(address)
print("Closing connection")
s.close()
