
import socket

GAMECONTROLLER_DATA_PORT = 3838
num_bits = 8
protocol_version = 0

#unknown: -1
gameType = {
	0:'round_robin', 
	1:'playoff', 
	2:'drop_in', 
	255:'unknown'
}
 
gameStates = {
	0:'Initial',
	1:'Ready', 
	2:'Set', 
	3:'Playing', 
	4:'Finished', 
	255:'Impossible'
}

secondaryGameStates = {
	0:'Normal', 
	1:'Penalty_Shoot', 
	2:'Overtime', 
	3:'Timeout', 
	4:'Direct_Free_Kick', 
	5:'Indirect_Free_Kick', 
	6:'Penalty', 
	255:'unknown'
}

teamColors = {
	0:'blue', 
	1:'red', 
	2:'yellow', 
	3:'black', 
	4:'white', 
	5:'green', 
	6:'orange', 
	7:'purple', 
	8:'brown', 
	9:'gray', 
	255:'unknown'
}

penalties = {
	0:'None',
	1:'SPL_ILLEGAL_BALL_CONTACT', 
	2:'SPL_PLAYER_PUSHING',
	3:'SPL_ILLEGAL_MOTION_IN_SET',
	4:'SPL_INACTIVE_PLAYER',
	5:'SPL_ILLEGAL_DEFENDER',
	6:'SPL_LEAVING_THE_FIELD',
	7:'SPL_KICK_OFF_GOAL',
	8:'SPL_REQUEST_FOR_PICKUP',
	9:'SPL_COACH_MOTION',
	14:'SUBSTITUTE',
	15:'MANUAL',
	30:'HL_BALL_MANIPULATION',
	31:'HL_PHYSICAL_CONTACT',
	32:'HL_ILLEGAL_ATTACK',
	33:'HL_ILLEGAL_DEFENSE',
	34:'HL_PICKUP_OR_INCAPABLE',
	35:'HL_SERVICE'
}

subMode = {
	0:'ready',
	1:'prepare'
}

def parser(msg):
	print "parse received broadcast"
	rcv = dict()
	
	rcv['Header'] = msg[0:3]
	protocol_version = ord(msg[4]) * 16 **2 + ord(msg[5])
	#rcv['PacketNum'] = ord(msg[6])
	rcv['playersPerTeam'] = ord(msg[7])
	rcv['gameType'] = gameType[ord(msg[8])]
	rcv['state'] = gameStates[ord(msg[9]) ]
	rcv['firstHalf'] = ord(msg[10])
	rcv['kickOffTeam'] = ord(msg[11])
	rcv['secondaryState'] = secondaryGameStates[ord(msg[12])]
	
	#NOT COMPLATED YET !!!!
	rcv['secondaryGameStates_teamPerforming_FK'] = ord(msg[13]) 
	rcv['secondaryGameStates_subMode'] = subMode[ord(msg[14])] 
	#rcv['secondaryGameStates_teamPerforming_FK'] = ord(msg[13]) 
	#rcv['secondaryGameStates_teamPerforming_FK'] = ord(msg[13])   #cannot find...  
	
	rcv['dropInTeam'] = ord(msg[17])
	
	#in seconds
	rcv['dropInTime'] = ord(msg[18]) * (16**2) + ord(msg[19])
	rcv['secsRemaining'] = ord(msg[20]) * (16**2) + ord(msg[21])
	rcv['secondaryTime'] = ord(msg[22]) * (16 **2) + ord(msg[23])
	
	#team info
	for j in range(2):
		tmpstr_team = 'Team%d_'%(j+1)
		rcv[tmpstr_team+ 'teamNumber'] = ord(msg[24 + j*308])
		rcv[tmpstr_team+'teamColour'] = teamColors[ord(msg[25+ j*308])]
		rcv[tmpstr_team+'score'] = ord(msg[26+ j*308]) << 8
		rcv[tmpstr_team+'penaltyShot'] = ord(msg[27+ j*308])
		
		rcv[tmpstr_team+'singleShots_int'] = ord(msg[28 + j*308]) * (16**2) + ord(msg[29 + j*308])
		rcv[tmpstr_team+'singleShots_strbin'] = "{0:b}".format(rcv[tmpstr_team+'singleShots_int'])
		
		rcv[tmpstr_team+'coachSequence'] = ord(msg[30+ j*308])
		rcv[tmpstr_team+'coachMessage'] = msg[31 + j*308:283 + j*308]   ###DEAL WITH LATER
		
		#robot info
		rcv[tmpstr_team+'coach_penalty'] = penalties[ord(msg[284 + j*308])]
		rcv[tmpstr_team+'coach_secsTillUnpenalised'] = ord(msg[285 + j*308])
		rcv[tmpstr_team+'coach_yellowCardCount'] = ord(msg[286 + j*308])
		rcv[tmpstr_team+'coach_redCardCount'] = ord(msg[287 + j*308])
		
		for i in range(11):
			tmpstr = tmpstr_team+'player%d_'%i
			rcv[tmpstr+'penalty'] = penalties[ord(msg[288 + i*4 + j*308])]
			rcv[tmpstr+'secsTillUnpenalised'] = ord(msg[289 + i*4 + j*308])
			rcv[tmpstr+'yellowCardCount'] = ord(msg[290 + i*4 + j*308])
			rcv[tmpstr+'redCardCount'] = ord(msg[291 + i*4 + j*308])
			
	return rcv
	

def broadcastReceiver():
	print "receive broadcast from game controller"
	sct = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sct.bind(('',GAMECONTROLLER_DATA_PORT))
	m=sct.recvfrom(1024)
	print len(m)
	rcv = parser(m[0])
	print rcv
	


if __name__ == '__main__':
	while True:
		broadcastReceiver()
