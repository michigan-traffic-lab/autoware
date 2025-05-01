float64 timestamp 			# time
int32 	count               # keep as default
float64 brake_cmd 			# [0.0, 1.0]
float64 throttle_cmd 		# [0.0, 1.0]
float64 steering_cmd 		# [-2.5pi, 2.5pi]
int16 	gear_cmd            # see below
int16 	turn_signal_cmd     # see below
 
#define of gears
#NONE   	=0
#PARK       =1
#REVERSE    =2
#NEUTRAL    =3
#DRIVE      =4
#LOW        =5

#define turn signal
#NONE    =0
#LEFT    =1
#RIGHT   =2