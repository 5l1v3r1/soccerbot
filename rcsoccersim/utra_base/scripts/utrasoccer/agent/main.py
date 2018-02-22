from network.udpclient import *
import time


import random
myPort = random.randint(0,1000)+2000
c = UDPClient('',myPort,'localhost',6000)

TEAM = input('Team: ')
VERSION = 7

c.sendToServer('(init '+TEAM+' (version '+str(VERSION)+'))')

# get the ack
l = c.read()
print(l)
time.sleep(1)

# ask player to start match
START = input('Start match? (y/n): ')
if START=='y':
    c.sendToServer('??start match??')

while True:
    print('\n\n\n\n\n\n\n')
    time.sleep(2)
    print(c.read())