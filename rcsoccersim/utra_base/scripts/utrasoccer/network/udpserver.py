import socket
from network.serializer import _unbyte_utf8, _byte_utf8

class UDPServer:
    ip = ''
    port = 1996
    timeout = 0.005
    buffSize = 8192
    packSize = 4096
    
    sock = None
    
    def __init__(self,_ip,_port):
        self.ip = _ip
        self.port = _port
        self.open()
        self.bind()
        
    def setPort(self,_port):
        self.port = _port
        
    def setIP(self,_ip):
        self.ip = _ip
    
    def setPackSize(self,_size):
        self.packSize = _size
    
    def setTimeout(self,_to):
        self.timeout = _to
    
    def setBuffSize(self,_size):
        self.buffSize = _size

    def open(self):
        try:
            self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        except socket.error as e:
            print(e)
        
    def close(self):
        try:
            self.sock.close()
        except:
            pass
        
    def bind(self):
        try:
            self.sock.bind((self.ip,self.port))
            self.sock.settimeout(self.timeout)    
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.buffSize)    
        except socket.error as e:
            print(e)
            
    def read(self):
        dict = {}
        try:
            while True:
                data, addr = self.sock.recvfrom(self.packSize)
                ddata = _unbyte_utf8(data)
                if dict.__contains__(addr):
                    dict[addr].append(ddata)
                else:
                    dict[addr] = [ddata]
        except socket.timeout:
            pass
        return dict
        
    def send(self,data,addr):
        edata = _byte_utf8(data)
        try:
            self.sock.sendto(edata,addr)
        except socket.error as e:
            print(e)

# Example usage 1, see udpclient for counterpart
# s = UDPServer('',1996)
# while True:
#     time.sleep(5)
#     l = s.read()  
#     if len(l) > 0:
#         print(l)  
#     for k in l:
#         s.send('ok',k)  