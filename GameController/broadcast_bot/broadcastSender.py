
import socket
from time import sleep

GAMECONTROLLER_RETURN_PORT = 3939

### TEMPORARY ###
protocol_version = 12

playerResponses = {
	"MAN_PENALIZE": 0,
	"MAN_UNPENALIZE": 1,
	"ALIVE": 2,
}

def parser_send(team_num,player_num,message):
	print "parse msg to send"
	
	msg = "RGrt"
	msg += chr(protocol_version)
	msg += chr(team_num)
	msg += chr(player_num)
	if((playerResponses.keys()).count(message) > 0):
		msg += chr(playerResponses[message])
	else:
		msg += chr(255) #unknown
	
	return msg
	
def broadcastSender():
	print "broadcast response"
	msg_send = parser_send(1,2,2)
	
	sct = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sct.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) 
	sct.sendto(msg_send,('<broadcast>',GAMECONTROLLER_RETURN_PORT))
	sct.close()
	
	
if __name__ == "__main__":
	while True:
		broadcastSender()
		sleep(1)
