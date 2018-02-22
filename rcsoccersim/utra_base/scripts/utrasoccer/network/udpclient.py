import socket
from network.serializer import _byte_utf8, _unbyte_utf8

class UDPClient:
    ip = ''
    port = 2000
    timeout = 0.005
    buffSize = 8192
    packSize = 4096
    
    sIP = 'localhost'
    sPort = 6000
        
    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    
    def __init__(self,_ip,_port,_sIP,_sPort):
        self.sIP = _sIP
        self.sPort = _sPort
        self.ip = _ip
        self.port = _port
        self.open()
        self.bind()
    
    def setPort(self,_port):
        self.port = _port
        
    def setIP(self,_ip):
        self.ip = _ip
        
    def setSPort(self,_port):
        self.sPort = _port
        
    def setSIP(self,_ip):
        self.sIP = _ip        
    
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
            print("error closing socket...\n")
            pass
        
    def bind(self):
        try:
            self.sock.bind((self.ip,self.port))
            self.sock.settimeout(self.timeout)    
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.buffSize)    
        except socket.error as e:
            print(e)
            
    def read(self):
        _dict = {}
        try:
            while True:
                data, addr = self.sock.recvfrom(self.packSize)
                ddata = _unbyte_utf8(data)
                if _dict.__contains__(addr):
                    _dict[addr].append(ddata)
                else:
                    _dict[addr] = [ddata]
        except socket.timeout:
            pass
        return _dict
        
    def send(self,data,addr):
        edata = _byte_utf8(data)
        try:
            self.sock.sendto(edata,addr)
        except socket.error as e:
            print(e)
            
    def sendToServer(self,data):
        edata = _byte_utf8(data)serialize
        try:
            self.sock.sendto(edata,(self.sIP,self.sPort))
        except socket.error as e:
            print(e)
    
# Example usage, launch rcssserver then execute code
#     Wait for some input before showing server response
#     To initialize put: (init myteam (version 7))
#     Hit enter again to see server output
# import random
# myport = random.randint(0,1000)+2000
# c = UDPClient('',myport,'localhost',6000)
# while True:
# 
#     msg = input('>> ')
#     c.sendToServer(msg)
#     l = c.read()
#     print(l)

