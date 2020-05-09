import math



L, R = 0, 1
delta_t = 1 # 1sec
TWO_PI = math.pi
LIN = 0
ANG = 1


ROT = 0
TRANS = 1
TIME_IND = 2
LAST_STATUS = 'LAST_STATUS'


GOAL_THRESH = 0.2
DELTA_T_LIN = 2 # in seconds
DELTA_T_ANG = 1


DONE = 'DONE'
MOVING = 'MOVING'
PATH_COVERED = 'PATH'
OBS_DETECTED = 'Obstacle Detected! Cheking obstruction..'
OBST_PRESENT = 'Path obstructed. Re-configuring path..'
BOT_MOVING = 'Bot Moving towards the goal...'
NO_OBST = 'No obstruction present. Resuming...'

GRAPH_CLR = 'lightskyblue'
PATH_CLR = 'blue'
TRACK_CLR = 'orange'