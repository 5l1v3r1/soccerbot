from udpclient import *


import random
myport = random.randint(0,1000)+2000
c = UDPClient('',myport,'localhost',6000)
while True:
 
    msg = input('>> ')
    c.sendToServer(msg)
    l = c.read()
    print(l)